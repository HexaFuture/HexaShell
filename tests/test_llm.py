"""The gateway client (runtime/llm.py) against a local fake gateway: a streamed reply must come out
exactly like the same reply sent whole, and a stream that fails is retried like any failed request."""

from __future__ import annotations

import http.server
import json
import threading
import time

from hexashell.runtime import llm

SSE, JSON = "text/event-stream; charset=utf-8", "application/json"
TEXT = "Hello world   é"  # U+2028 is a line break to str.splitlines(), not to the stream
CALLS = [{"id": "call_a", "type": "function", "function": {"name": "python", "arguments": '{"code": "print(1)"}'}},
         {"id": "call_b", "type": "function", "function": {"name": "finish", "arguments": '{"status": "done"}'}}]
USAGE = {"prompt_tokens": 10, "completion_tokens": 5, "prompt_tokens_details": {"cached_tokens": 4},
         "completion_tokens_details": {"reasoning_tokens": 2}}


class FakeGateway:
    """Answers each POST with the next scripted (status, content type, body pieces[, Content-Length]);
    a float among the pieces is a pause of that many seconds."""

    def __init__(self):
        self.script: list[tuple[int, str, list[bytes]]] = []
        self.bodies: list[dict] = []
        outer = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                outer.bodies.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
                status, kind, pieces, *length = outer.script.pop(0)
                self.send_response(status)
                self.send_header("Content-Type", kind)
                if length:
                    self.send_header("Content-Length", str(length[0]))
                self.end_headers()
                for piece in pieces:
                    if isinstance(piece, float):
                        time.sleep(piece)
                        continue
                    self.wfile.write(piece)
                    self.wfile.flush()

            def log_message(self, *args):
                pass

        self.httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}/v1"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def _sse(*chunks, done=True, size=7) -> list[bytes]:
    """Events as a gateway streams them, cut into small pieces (lines and characters split)."""
    text = ": keep-alive\n\n" + "".join(f"data: {json.dumps(c, ensure_ascii=False)}\n\n" for c in chunks)
    raw = (text + ("data: [DONE]\n\n" if done else "")).encode("utf-8")
    return [raw[i:i + size] for i in range(0, len(raw), size)]


def _delta(**delta) -> dict:
    return {"choices": [{"index": 0, "delta": delta}]}


STREAM = _sse(
    _delta(role="assistant", content=""), _delta(reasoning_content="think "), _delta(reasoning_content="more"),
    _delta(content="Hello "), _delta(content=TEXT[len("Hello "):]),
    _delta(tool_calls=[{"index": 0, "id": "call_a", "type": "function",
                        "function": {"name": "python", "arguments": ""}}]),
    _delta(tool_calls=[{"index": 0, "function": {"arguments": '{"code": "print(1)'}}]),
    _delta(tool_calls=[{"index": 0, "function": {"arguments": '"}'}}]),
    _delta(tool_calls=[{"index": 1, "id": "call_b", "type": "function",
                        "function": {"name": "finish", "arguments": '{"status": '}}]),
    _delta(tool_calls=[{"index": 1, "function": {"arguments": '"done"}'}}]),
    {"choices": [{"index": 0, "delta": {}, "finish_reason": "tool_calls"}]},
    {"choices": [], "usage": USAGE})
WHOLE = [json.dumps({"choices": [{"index": 0, "finish_reason": "tool_calls", "message": {
    "role": "assistant", "content": TEXT, "reasoning_content": "think more", "tool_calls": CALLS}}],
    "usage": USAGE}).encode()]


def _same(a: llm.Reply, b: llm.Reply) -> None:
    assert (a.text, a.calls, a.finish_reason, a.usage, a.reasoning) == \
        (b.text, b.calls, b.finish_reason, b.usage, b.reasoning)


def test_streamed_reply_equals_whole_reply():
    fake = FakeGateway()
    events = []
    streamed = llm.Gateway(fake.url, "k", "m", log=events.append)
    whole = llm.Gateway(fake.url, "k", "m", stream=False)
    try:
        fake.script[:] = [(200, SSE, STREAM), (200, JSON, WHOLE), (200, JSON, WHOLE)]
        a = streamed.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        b = whole.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        c = streamed.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)  # sent whole
        assert a.text == TEXT and a.calls == CALLS and a.reasoning == "think more"
        assert a.usage == {"prompt": 10, "completion": 5, "cached": 4, "reasoning": 2}
        _same(a, b)
        _same(c, b)
        assert fake.bodies[0]["stream"] is True and fake.bodies[0]["stream_options"] == {"include_usage": True}
        assert "stream" not in fake.bodies[1] and "stream_options" not in fake.bodies[1]
        assert streamed.settings()["stream"] is True and whole.settings()["stream"] is False
        assert events[0]["status"] == 200 and events[0]["first_s"] >= 0 and "first_s" not in events[1]
    finally:
        streamed.close()
        whole.close()
        fake.close()


def test_stream_failures_are_retried():
    fake = FakeGateway()
    events = []
    gw = llm.Gateway(fake.url, "k", "m", log=events.append)
    saved = llm.BILLING_PAUSE, llm.RETRY_PAUSE
    llm.BILLING_PAUSE = llm.RETRY_PAUSE = (0.01, 1.0, 0.01)
    try:
        broke = json.dumps({"error": {"message": "insufficient balance", "type": "billing_error"}}).encode()
        fake.script[:] = [
            (403, JSON, [broke]),
            (200, SSE, _sse(_delta(content="Hel"), done=False)),  # cut off: no finish reason
            (200, SSE, _sse(_delta(content="Hel"), {"error": {"message": "upstream failed"}})),
            (200, SSE, STREAM)]
        reply = gw.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        assert reply.text == TEXT and reply.calls == CALLS and reply.attempts == 4
        assert [e["status"] for e in events] == [403, 0, 0, 200]
        assert "broke off" in events[1]["error"] and "upstream failed" in events[2]["error"]
        assert gw.lost_s > 0
    finally:
        llm.BILLING_PAUSE, llm.RETRY_PAUSE = saved
        gw.close()
        fake.close()


def test_stream_is_read_to_its_end():
    """The client keeps reading after [DONE] until the server ends the body (closing earlier makes the
    gateway log the stream as failed); a tail that breaks after [DONE] leaves the answer whole."""
    fake = FakeGateway()
    events = []
    gw = llm.Gateway(fake.url, "k", "m", log=events.append)
    try:
        fake.script[:] = [(200, SSE, STREAM + [0.5, b": bye\n\n"]),
                          (200, SSE, STREAM, sum(map(len, STREAM)) + 100)]  # body ends 100 bytes short
        t0 = time.monotonic()
        a = gw.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        assert time.monotonic() - t0 >= 0.5
        b = gw.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        for reply in (a, b):
            assert reply.text == TEXT and reply.calls == CALLS and reply.attempts == 1
        assert [e["status"] for e in events] == [200, 200]
    finally:
        gw.close()
        fake.close()


def _events(*events) -> list[bytes]:
    """Messages API events as they stream (named events, cut into small pieces)."""
    text = "".join(f"event: {e['type']}\ndata: {json.dumps(e, ensure_ascii=False)}\n\n" for e in events)
    raw = text.encode("utf-8")
    return [raw[i:i + 7] for i in range(0, len(raw), 7)]


CLAUDE_STREAM = _events(
    {"type": "message_start", "message": {"model": "claude-opus-5-5", "usage": {
        "input_tokens": 7, "cache_read_input_tokens": 100, "cache_creation_input_tokens": 20}}},
    {"type": "content_block_start", "index": 0, "content_block": {"type": "thinking", "thinking": "", "signature": ""}},
    {"type": "content_block_delta", "index": 0, "delta": {"type": "thinking_delta", "thinking": "Look first, "}},
    {"type": "content_block_delta", "index": 0, "delta": {"type": "thinking_delta", "thinking": "then move."}},
    {"type": "content_block_delta", "index": 0, "delta": {"type": "signature_delta", "signature": "c2ln"}},
    {"type": "content_block_stop", "index": 0},
    {"type": "ping"},
    {"type": "content_block_start", "index": 1, "content_block": {"type": "text", "text": ""}},
    {"type": "content_block_delta", "index": 1, "delta": {"type": "text_delta", "text": TEXT}},
    {"type": "content_block_stop", "index": 1},
    {"type": "content_block_start", "index": 2,
     "content_block": {"type": "tool_use", "id": "toolu_a", "name": "python", "input": {}}},
    {"type": "content_block_delta", "index": 2, "delta": {"type": "input_json_delta", "partial_json": '{"code": "pr'}},
    {"type": "content_block_delta", "index": 2, "delta": {"type": "input_json_delta", "partial_json": 'int(1)"}'}},
    {"type": "content_block_stop", "index": 2},
    {"type": "message_delta", "delta": {"stop_reason": "tool_use"},
     "usage": {"output_tokens": 40, "output_tokens_details": {"thinking_tokens": 25}}},
    {"type": "message_stop"})


def test_claude_reply_and_history():
    """A streamed Messages answer comes out like a chat-completions one, and its content blocks (the
    signed thinking among them) go back unchanged in the next request."""
    from hexashell.runtime.conversation import Conversation

    fake = FakeGateway()
    events = []
    gw = llm.AnthropicGateway(fake.url, "k", "claude-opus-5-5", reasoning_effort="medium", log=events.append)
    tools = [{"type": "function", "function": {"name": "python", "description": "Run code",
                                               "parameters": {"type": "object", "properties": {}}}}]
    conv = Conversation("rules")
    pic = conv.picture(__import__("numpy").zeros((4, 4, 3), "uint8"), "head at the start", pinned=True)
    conv.user("task card", [pic])
    try:
        fake.script[:] = [(200, SSE, CLAUDE_STREAM), (200, SSE, CLAUDE_STREAM)]
        reply = gw.ask(conv.wire(), tools, deadline=time.monotonic() + 30)
        assert reply.text == TEXT and reply.finish_reason == "tool_calls"
        assert reply.calls == [{"id": "toolu_a", "type": "function",
                                "function": {"name": "python", "arguments": '{"code": "print(1)"}'}}]
        assert reply.reasoning == "Look first, then move."
        assert reply.usage == {"prompt": 127, "completion": 40, "cached": 100, "reasoning": 25}
        assert events[0]["model"] == "claude-opus-5-5"
        first = fake.bodies[0]
        assert first["thinking"] == {"type": "adaptive", "display": "summarized"}
        assert first["output_config"] == {"effort": "medium"} and first["stream"] is True
        assert first["tools"] == [{"name": "python", "description": "Run code",
                                   "input_schema": {"type": "object", "properties": {}}}]
        assert first["system"] == [{"type": "text", "text": "rules", "cache_control": {"type": "ephemeral"}}]
        image = first["messages"][0]["content"][2]
        assert image["type"] == "image" and image["source"]["media_type"] == "image/jpeg"

        conv.assistant(reply.text, reply.calls, reply.raw_message["_blocks"])
        conv.tool("toolu_a", "1")
        conv.user("Pictures from robot.look() in this turn:", [conv.picture(
            __import__("numpy").zeros((4, 4, 3), "uint8"), "look: head")])
        gw.ask(conv.wire(), tools, deadline=time.monotonic() + 30)
        second = fake.bodies[1]["messages"]
        assert [m["role"] for m in second] == ["user", "assistant", "user"]
        assert second[1]["content"] == [
            {"type": "thinking", "thinking": "Look first, then move.", "signature": "c2ln"},
            {"type": "text", "text": TEXT},
            {"type": "tool_use", "id": "toolu_a", "name": "python", "input": {"code": "print(1)"}}]
        kinds = [b["type"] for b in second[2]["content"]]
        assert kinds == ["tool_result", "text", "text", "image"]
        assert second[2]["content"][-1]["cache_control"] == {"type": "ephemeral"}
        assert "cache_control" not in conv.messages[-1]["content"][-1]  # the conversation itself is untouched
    finally:
        gw.close()
        fake.close()


def test_chinese_empty_account_waits_only_on_claude():
    """The Claude relays say "预扣费额度失败" when the account is empty: the Claude gateway waits and
    retries; the chat-completions gateway keeps treating such a 403 as a refusal, as before."""
    fake = FakeGateway()
    claude = llm.AnthropicGateway(fake.url, "k", "claude-opus-5-5")
    chat = llm.Gateway(fake.url, "k", "m")
    saved = llm.BILLING_PAUSE
    llm.BILLING_PAUSE = (0.01, 1.0, 0.01)
    broke = json.dumps({"error": {"type": "new_api_error", "message": "预扣费额度失败"}}, ensure_ascii=False).encode()
    try:
        fake.script[:] = [(403, JSON, [broke]), (200, SSE, CLAUDE_STREAM), (403, JSON, [broke])]
        assert claude.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30).attempts == 2
        try:
            chat.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        except llm.GatewayError as exc:
            assert exc.kind == "refused" and exc.status == 403
        else:
            raise AssertionError("the chat gateway should refuse")
    finally:
        llm.BILLING_PAUSE = saved
        claude.close()
        chat.close()
        fake.close()


def test_claude_cache_breakpoints():
    """Breakpoints at the end of the system prompt, of the previous request (the turn before the latest
    reply) and of the conversation: a turn with many pictures adds more than the 20 blocks a breakpoint
    looks back for an earlier cache entry."""
    call = {"id": "t1", "type": "function", "function": {"name": "python", "arguments": "{}"}}
    image = {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,AAAA"}}
    messages = [{"role": "system", "content": "rules"}, {"role": "user", "content": "card"},
                {"role": "assistant", "content": "", "tool_calls": [call]},
                {"role": "tool", "tool_call_id": "t1", "content": "ok"},
                {"role": "user", "content": [{"type": "text", "text": "Pictures:"}] + [image] * 12}]
    system, turns = llm.to_anthropic(messages)
    marked = [(t, b) for t, turn in enumerate(turns) for b, block in enumerate(turn["content"])
              if "cache_control" in block]
    assert system[-1]["cache_control"] == {"type": "ephemeral"}
    assert marked == [(0, 0), (2, 13)]
    assert "cache_control" not in json.dumps(messages)
    _, first = llm.to_anthropic(messages[:2])  # the first request: one turn, one mark
    assert first == [{"role": "user", "content": [{"type": "text", "text": "card",
                                                   "cache_control": {"type": "ephemeral"}}]}]


def test_claude_unsigned_thinking_is_not_sent_back():
    """A reply cut off while thinking (max_tokens) leaves a thinking block without a signature: it is
    kept as the reply's reasoning, but not among the blocks sent back (the API would refuse it)."""
    fake = FakeGateway()
    gw = llm.AnthropicGateway(fake.url, "k", "claude-opus-5-5")
    cut = _events(
        {"type": "message_start", "message": {"model": "claude-opus-5-5", "usage": {"input_tokens": 7}}},
        {"type": "content_block_start", "index": 0, "content_block": {"type": "thinking", "thinking": "", "signature": ""}},
        {"type": "content_block_delta", "index": 0, "delta": {"type": "thinking_delta", "thinking": "A long thought"}},
        {"type": "message_delta", "delta": {"stop_reason": "max_tokens"}, "usage": {"output_tokens": 8192}},
        {"type": "message_stop"})
    try:
        fake.script[:] = [(200, SSE, cut)]
        reply = gw.ask([{"role": "user", "content": "hi"}], [], deadline=time.monotonic() + 30)
        assert reply.finish_reason == "length" and reply.reasoning == "A long thought"
        assert reply.text == "" and reply.calls == [] and reply.raw_message["_blocks"] == []
    finally:
        gw.close()
        fake.close()
