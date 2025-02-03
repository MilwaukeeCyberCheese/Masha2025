# Milwaukee CyberCheese 2024-2025 Reefscape Code

When cloning this repository locally, make sure to setup the proper libraries for the [REV Distance Sensor](https://github.com/REVrobotics/2m-Distance-Sensor/releases)

Additionally, copy this script into the .git/hooks/pre-commit file to run spotless check automatically at commit

```
#!/bin/sh

./gradlew spotlessCheck
```
