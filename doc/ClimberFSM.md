# Climber FSM Spec

```mermaid
---
config:
  layout: elk
  title: Climber State Diagram
---
stateDiagram
  direction BT
  [*] --> I
  I --> I:!nextBtn
  MDC --> MDC:!manualOverrideBtn
  I --> MDC:manualOverrideBtn
  MDC --> I:manualOverrideBtn
  I --> L1E:nextBtn
  L1E --> L1E:!(nextBtn && isExtended())
  L1R --> L1R:!((nextBtn) && isLatched())
  L1E --> L1R:nextBtn && isExtended()
  L1R --> LF
  L1E --> I:emergencyAbort()
  L1R --> I:emergencyAbort()
  AD --> I:isOnGround()
  AU1 --> I:L1_EXTEND
  AU2 --> I:L1_RETRACT
  I --> AD1: autoDownBtn
  AD1 --> AD2: autoDownBtn
  I:IDLE
  MDC:MANUAL_DIRECT_CONTROL
  L1E:L1_EXTEND
  L1R:L1_RETRACT
  LF:LOCKED_FINAL
  AU1:AUTO_UP_ONE
  AU2:AUTO_UP_TWO
  AD1:AUTO_DOWN_ONE
  AD2:AUTO_DOWN_TWO
  