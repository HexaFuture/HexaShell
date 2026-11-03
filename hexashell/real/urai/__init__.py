"""HexaShell on a robot run by URAI, through URAI's HTTP service.

URAI plans and executes motions and reports robot state and RGB-D pictures. HexaShell keeps the
conversation, cells and records, displays images and computes pixel/world coordinates. This adapter
uses no segmentation service. There is no judge: the model's own report ends the episode
(:class:`.session.UraiSession`). Prompts, interface help and model-generated reports are English.

``python -m hexashell.real.urai --url http://<host>:<port> --task "..."`` runs one episode.
"""
