# Navigator producer/consumer protocol V5

This developer-only feature is **off by default**. It ports the bounded
Navigator coast-reference experiment into separately owned producer and consumer
histories. It is not a new steering or general longitudinal tune.

The startup selector is `NAVIGATOR_COAST_PRODUCTION=1`. Both peers must use the
paired source revision and supported Navigator Alpha coordination. This selector
also enables the reference policy; it cannot be combined with
`NAVIGATOR_RECEIPT_RETENTION=1`. No new Params key or settings toggle is added.
Existing history-release selection remains independent.

## Ownership

`LongControl.production` owns a separate nominal LongControl and a bounded
issued-request registry. It never assigns the actual PID integral. The consumer
owns the actual native gate, allocator, clearance observer and handoff, plus a
separate nominal reference. Requests and complete nominal feedback cross the
normal Cap'n Proto `carControl` / `carOutput` boundary; neither peer relies on a
shared Python object, research path or recorded PID value.

The wire binds declared protocol/source revisions, a positive producer-run
nonce, epoch/sequence, consumed target, immutable producer pose, independently
refreshed execution pose and exact actual/nominal raw values. Source identifiers
check compatible declared revisions; they do not cryptographically authenticate
local IPC or detect an arbitrarily modified same-ID checkout. Validation receipts
must bind exact source and build hashes separately.

## Bounds and retirement

Admission remains restricted to fresh, weak, no-lead requests at 20–30 mph with
actual measured brake clearance. A consumer history permits one nonrenewable
500 ms experimental episode. Native allocator, dose, response and safety limits
remain unchanged. Ordinary native 300 ms behavior remains available after trial
retirement. A new engagement cannot renew the experimental episode in that
consumer history.

Producer nominal consultation is separate from actor permission. An old receipt
selected before its original native 300 ms endpoint cannot unfreeze the integral
after that endpoint. Fresh later independent nominal facts remain separately
eligible, without extending the old receipt or granting actuator authority.
Actual release evidence may seed only native pending qualification after the
original trial deadline, fresh current checks and natural zero debt. No nominal
clearance, debt reset or state transplant is used.

Malformed declared V5 inputs use the existing coordinated fault path before CAN
construction. Unknown or older peer versions retain native fallback. Producer
restart while engaged faults; a disabled owner may establish a new producer run
without resetting the consumer's actual numerical history or renewing a spent
trial. All retained-proof math requires a fresh three-component finite pose.

## Validation limits

Default-off parity, serialized producer/consumer replay, adversarial wire and
restart tests, native Ford checks and both repository builds are separate gates.
Added schema defaults must be disclosed when comparing old decoded dictionaries;
CAN and legacy fields must still match. Longitudinal replays with lateral inactive
do not validate road steering. Host tests and frozen recorded inputs do not prove
closed-loop ECU response, loaded ARM deadlines, physical smoothness or road safety.

Device staging, parked checks and activation require their own authorization and
evidence. This source does not automatically enable the experiment on a device.
