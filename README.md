[![CI](https://github.com/MilwaukeeCyberCheese/Masha2025/actions/workflows/build.yml/badge.svg)](https://github.com/MilwaukeeCyberCheese/Masha2025/actions/workflows/build.yml)

# Milwaukee CyberCheese 2024-2025 Reefscape Code

When cloning locally, copy this script into the .git/hooks/pre-commit file to run spotless automatically at commit

```
#!/bin/sh

./gradlew spotlessCheck
```
