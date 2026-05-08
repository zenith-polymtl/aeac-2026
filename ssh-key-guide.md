# SSH key setup — Windows PowerShell

## 1. Generate a key on the computer

```powershell
ssh-keygen -t ed25519 -C "framework-zenith"
```

You can press Enter to keep the default save location.

## 2. Copy the public key to the robot

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh zenith@jetson-hexa.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

### What to change

Change this part:
```
framework-zenith
```

to a name that identifies the computer/user, for example:
```
framework-colin
```

Change this part:
```
zenith@jetson-hexa.local
```

to the robot login:
```
username@robot-address
```

### Examples

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh ubuntu@192.168.1.50 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh zenith@robot2.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

### Test

```powershell
ssh zenith@jetson-hexa.local
```