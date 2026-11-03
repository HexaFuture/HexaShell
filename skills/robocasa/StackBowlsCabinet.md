# StackBowlsCabinet
## Judging
- Done when both bowls are inside one level of the open wall cabinet (between a shelf and the
  one above; a little front overhang is fine), one sitting in the other, roughly centred on it,
  and the hand well away from both, at once.
- Still running after the last release: a bowl fell out, leans or sits on a rim; or hand near.
## Flow
1. Priors: both bowls on the counter in front, the cabinet above (9 of 10 eps). Shelf ~0.5 m
   above the counter (8 eps), level 21-27 cm tall (5 eps).
2. Set the smaller bowl in the larger one on the counter, then carry the two in at once (prior
   tool; scripted). Face the larger bowl, ~0.6 m off: `robot.rim_pick(large, outer=True,
   dry_run=True)` gives its rim's radius. Face the smaller one: `robot.rim_pick(small)`; with
   pitch=60, under=0.01 if it closed on nothing or the larger bowl stands just short of it.
   Smaller one out of reach at first: early on, take the larger in alone, then the smaller into it
   (two trips: scripted 5/10; 1 ep switched late and ran out of steps).
3. Hand back to ~0.3 m ahead, the held bowl's bottom ~8 cm over the larger rim; base back to
   where it faced the larger bowl; hand ~the held radius short of the larger middle; down till
   ~1 cm over the larger rim; open; hand 6 cm back along the fingers, up 8 cm, home (scripted).
4. `robot.rim_pick(large, outer=True, under=0.02)` takes the pair by the larger rim; closed on
   under ~1.5 cm: set it down, pinch again 0.5 cm deeper (a 1.2 cm hold slipped; scripted).
5. Hand back to ~0.3 m ahead, low; base back till the pair's far side is ~4 cm short of the
   cabinet front. The opening's middle >10 cm aside, in a free stretch at least the pair's
   width (narrower: BLOCKED): drive the base across; no such stretch seen: straight in (scripted).
6. Raise the larger bowl's bottom (~0.75 x its height under the hand) to ~3 cm over the shelf;
   pitch 25; base forward the same; hand ~3 cm past the front edge (from where the base is now);
   down till it stops (8 cm at most); open; hand 13 cm back along the fingers; home (scripted).

## Verified parameters
- rim_pick of a bowl closed on 0.8-1.9 cm; of the pair, outer=True: 1.6-5.6 cm (scripted).
- Moving the raised pair 0.2 m across with the hand, not the base, went in fine (scripted).

## Common failures and fixes
- Plain rim_pick of the pair pinched the smaller bowl's rim (higher, a little further in) and
  lifted it alone or slipped (2 of 9 scripted): use outer=True.
- Larger bowl just short of the smaller: a pitch-45 hand came down on its rim; carried ~4 cm
  over it, the held smaller bowl sagged ~2 cm and shoved it away (scripted).
- Let go short of the front edge (the base had moved since): tipped back out (scripted).
- Base BLOCKED backing off: draw the hand in by what it is short. Once the arm could not take
  it in that far: going up, the far side hit the cabinet and the pair fell (scripted).
- From v16 runs: open only when the pictures show the whole bowl inside (2 eps); entering high
  was BLOCKED at the level's top: go 2-3 cm lower (2 eps).

## Typical step costs
- Horizon 2100. Scripted: ~850-1150 to the release, ~1500 with a second pinch (scripted).
