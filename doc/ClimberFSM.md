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
  L2E --> L2E:!((nextBtn) && isExtended())
  L2R --> L2R:!((nextBtn) && isLatched())
  L3E --> L3E:!((nextBtn) && isExtended())
  L3R --> L3R:!(nextBtn)
  LF --> LF
  L1E --> L1R:nextBtn && isExtended()
  L1R --> L2E:(nextBtn) && isLatched()
  L2E --> L2R:(nextBtn) && isExtended()
  L2R --> L3E:(nextBtn) && isLatched()
  L1E --> I:emergencyAbort()
  L1R --> I:emergencyAbort()
  L2E --> I:emergencyAbort()
  L2R --> I:emergencyAbort()
  L3E --> I:emergencyAbort()
  L3R --> I:emergencyAbort()
  L3E --> L3R:(nextBtn) && isExtended()
  L3R --> LF:isLatched()
  I --> AD
  AD --> I:isOnGround()
  AU1 --> I:L1_EXTEND
  AU2 --> I:L1_RETRACT
  AD --> AD:!isOnGround()
  I:IDLE
  MDC:MANUAL_DIRECT_CONTROL
  L1E:L1_EXTEND
  L1R:L1_RETRACT
  L2E:L2_EXTEND
  L2R:L2_RETRACT
  L3E:L3_EXTEND
  L3R:L3_RETRACT
  LF:LOCKED_FINAL
  AU1:AUTO_UP_ONE
  AI:AUTO_IDLE
  AD:AUTO_DOWN
  AU2:AUTO_UP_TWO