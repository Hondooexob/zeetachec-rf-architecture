# Protocol Lock

The RF packet format is locked:

SOF | Version | Type | Src | Dst | Seq | Len | Payload | CRC16

Do not change:
- message IDs
- payload semantics
- reliable message rules

without explicit approval.
