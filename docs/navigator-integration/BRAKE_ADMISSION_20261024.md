# Bounded brake admission integration — October 24, 2026

Integrates the frozen, reviewed brake-release prototype into the paired Navigator branches. A new mild negative grade-compensated request with positive planner acceleration can briefly coast while qualification accumulates. Sustained demand, larger braking, existing braking and excluded operating modes retain their baseline path. Qualification is bounded by dwell and negative command exposure; propulsion is withheld during qualification.

The received-torque observer reopens brake-clearance qualification when measured torque remains above threshold after prior clearance. Invalid/reversed feedback faults; the existing clearance samples and one-second release timeout remain. This does not assume brake release from absence of a command.

Owner 1.20/0.97, Ford geometry, lateral code and native safety limits are unchanged. Coordination remains default-off. Local evidence is recorded in the Mac workspace's `docs/research/navigator-brake-release-integration-2026-10-24` report: exact prototype AST equivalence, actual integrated contracts, independent review, build/test receipts and active-path saved-drive replay. Retained 1,066-segment historical replay binds to the identical prototype implementation and is not a fresh road or active-longitudinal efficacy test.

The original frozen torque response still produces the release-timeout episode. Command-dependent models support avoiding its triggering pulse, but physical resolution and improved ride quality remain unproven. No installation, ARM build, parked checks or road validation are claimed by this local integration.
