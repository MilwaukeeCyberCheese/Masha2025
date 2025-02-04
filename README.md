# Milwaukee CyberCheese 2024-2025 Reefscape Code

When cloning locally, copy this script into the .git/hooks/pre-commit file to run spotless automatically at commit

```
#!/bin/sh

./gradlew spotlessCheck
```
