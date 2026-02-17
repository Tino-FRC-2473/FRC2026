# Climber FSM Spec

```mermaid
---
config:
  title: Climber State Diagram
---
stateDiagram
  [*] --> AI
  AI --> I: emergencyAbortBtnPressed
  AI --> AD1: nextBtn
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
  AD1 --> AD2: autoDownBtnPressed
  AD1 --> AD1: !autoDownBtnPressed

  I:IDLE
  AI:AUTO_IDLE
  MDC:MANUAL_DIRECT_CONTROL (Driver has control)
  L1E:L1_EXTEND (Climber extends to L1 Height)
  L1R:L1_RETRACT (Climber retracts to L1 Height)
  LF:LOCKED_FINAL (Can't switch from this state, meant for end of robot match)
  AU1:AUTO_UP_ONE (First part of Auto up, extend)
  AU2:AUTO_UP_TWO (Second part of Auto up, retract)
  AD1:AUTO_DOWN_ONE (First part of Auto down, extend)
  AD2:AUTO_DOWN_TWO (Second part of Auto down, retract)
  