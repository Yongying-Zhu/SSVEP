# 6s Swift

Timed switch recordings use one numbered directory per trial:

```text
6s_swift/
├── 6s_swift_01/
├── 6s_swift_02/
└── ...
```

Within each trial, `t=0` is the first valid `/ssvep/command` received by the
temporary stimulus. The stimulus then runs `forward`, `backward`, `left`,
`right`, and `stop` for 6 seconds each. The `/ssvep/sequence_event` topic
records the first-valid marker and phase boundaries.
