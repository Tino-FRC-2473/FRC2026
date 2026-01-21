# Climber FSM Spec

```mermaid
---
config:
  title: Climber State Diagram
  layout: elk
---
stateDiagram
  [*] --> I
  I --> I:!(nextBtn || manualOverrideBtnReleased || autoDownBtnPressed)
  MDC --> MDC:!manualOverrideBtnReleased
  I --> MDC:manualOverrideBtnPressed
  MDC --> I:manualOverrideBtnReleased
  I --> L1E:nextBtn
  L1E --> L1E:!(nextBtn && isExtended())
  L1R --> L1R:!((nextBtn) && isLatched())
  L1E --> L1R:nextBtn && isExtended()
  L1R --> LF
  L1E --> I:emergencyAbortBtnPressed
  L1R --> I:emergencyAbortBtnPressed
  AD2 --> I:isOnGround()
  AD2 --> AD2: !isOnGround()
  AU1 --> I:L1_EXTEND
  AU2 --> I:L1_RETRACT
  I --> AD1: autoDownBtnPressed
  AD1 --> AD2: autoDownBtnPressed
  AD1 --> AD1: !autoDownBtnPressed

  I:IDLE
  MDC:MANUAL_DIRECT_CONTROL
  L1E:L1_EXTEND
  L1R:L1_RETRACT
  LF:LOCKED_FINAL
  AU1:AUTO_UP_ONE
  AU2:AUTO_UP_TWO
  AD1:AUTO_DOWN_ONE
  AD2:AUTO_DOWN_TWO
  