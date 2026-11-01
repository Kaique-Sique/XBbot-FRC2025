# XBbot — FRC 2025 REEFSCAPE

Robot software for XBbot's **2025 FIRST Robotics Competition (REEFSCAPE)** season.

## Planned Mechanisms

1. **Passive coral intake ramp** — an articulated ramp angled toward the coral station.
2. **Coral manipulator** — an articulated mechanism with powered wheels.
3. **Elevator** — a lifting mechanism.
4. **Algae intake** — an articulated mechanism with powered wheels.

Sensors, actuators, motor controllers, CAN IDs, mechanical limits, and tuning parameters have not been specified yet. Do not assume or invent hardware details or values.


## Getting Started

1. Clone this repository.
2. Install the WPILib 2025 development environment and a compatible JDK (Java 17).
3. Make sure the Gradle Wrapper files are present in the repository:
   - `gradlew`
   - `gradlew.bat`
   - `gradle/wrapper/gradle-wrapper.jar`
   - `gradle/wrapper/gradle-wrapper.properties`
4. On Linux or macOS, make the wrapper executable if needed:
   ```bash
   chmod +x gradlew
   ```
5. Run the wrapper from the repository root to download Gradle and resolve the project dependencies.

> **Note:** The Gradle Wrapper files have not been added to this repository yet. The commands below document the intended workflow and will work once a WPILib robot project and its wrapper have been added.

## Common Gradle Commands

Run these commands from the repository root.

>[!INFO]
>
> JDK 17 or latest

| Task | Linux / macOS | Windows PowerShell |
|---|---|---|
| Build the project | `./gradlew build` | `.\gradlew.bat build` |
| Deploy to the robot | `./gradlew deploy` | `.\gradlew.bat deploy` |
| Run unit tests | `./gradlew test` | `.\gradlew.bat test` |
| List available tasks | `./gradlew tasks` | `.\gradlew.bat tasks` |
| Build without running tests | `./gradlew assemble` | `.\gradlew.bat assemble` |

For more detailed build output, append `--info` to a command.

## Development Environment

- **Recommended IDE:** Visual Studio Code with the WPILib extension.
- **Java:** JDK 17, matching the WPILib 2025 toolchain.
- **Build system:** Gradle through the project-provided Gradle Wrapper.
