# Shooter FSM Spec
```mermaid
---
config:
    title: Shooter State Diagram
---
stateDiagram-v2
    [*] --> Idle

    Idle --> Shooter_Prep : leftBumperPressed
    Idle --> Passer_Prep : rightBumperPressed
    Idle --> Manual_Prep : leftTriggerPressed

    Shooter_Prep --> Passer_Prep : rightBumperPressed
    Shooter_Prep --> Manual_Prep : leftTriggerPressed
    Shooter_Prep --> Idle : touchpadPressed
    Shooter_Prep --> Intake : hoodAtAngle && flywheelAtSpeed && rightTriggerPressed

    Passer_Prep --> Shooter_Prep : leftBumperPressed
    Passer_Prep --> Manual_Prep : leftTriggerPressed
    Passer_Prep --> Idle : touchpadPressed
    Passer_Prep --> Intake : hoodAtAngle && flywheelAtSpeed && rightTriggerPressed

    Manual_Prep --> Idle : touchpadPressed
    Manual_Prep --> Intake : hoodAtAngle && flywheelAtSpeed && rightTriggerPressed

    Intake --> Manual_Prep : (!hoodAtAngle || !flywheelAtSpeed || !rightTriggerPressed) && previousState = Manual_Prep
    Intake --> Passer_Prep : (!hoodAtAngle || !flywheelAtSpeed || !rightTriggerPressed) && previousState = Passer_Prep
    Intake --> Shooter_Prep : (!hoodAtAngle || !flywheelAtSpeed || !rightTriggerPressed) && previousState = Shooter_Prep
    Intake --> Idle : touchpadPressed
    Intake --> Intake : hoodAtAngle && flywheelAtSpeed && rightTriggerPressed
    

    note: Hood alignment logic runs continuously based on target selection when in a prep state
    note: 1. Intake is a single-shot state and exits immediately after a shot, or upon held inputs can be continued
    note: 2. 5 button inputs here, right trigger is always to shoot and touchpad is always to go to idle
    note: 3. Manual prep can only go to idle or intake because the button inputs to manually raise/lower hood are taken from the passer_prep and shooter_prep toggle states
    note: 4. Every prep state handles both hood alignment and flywheel speed. Intake state will only control the indexer speed to allow a ball to be indexed
    note: 5. All states will remain in their current state with no input, with the exception being the intake state
    note: 6. Every prep state will also maintain motor speed and continuously align hood to match new poses and desires
    note: 7. Passer Prep and Shooter Prep are autonomous alignment prep states to different targets. Shooter Prep will target the hub and Passer Prep will target predetermined passing points (currently set to the outpost)
    note: 8. Manual Prep will primarily be used for testing or as a backup if things go wrong in competitions
    note: 9. Auto States would follow the same functionality as Passer_Prep and Shooter_Prep (not Manual_Prep), the only difference would be the physical trigger for intaking or toggling states not being there

   