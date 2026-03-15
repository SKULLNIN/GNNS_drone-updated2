# Sync code and settings to your laptop

Use Git to push this repo (code + config) to GitHub, then pull on your laptop so everything stays in sync.

## 1. Push from this PC (after changes)

```bash
cd "C:\Users\guutu\OneDrive\Desktop\GNNS_drone-updated2"

# Add all code and config (scripts, config/*.yaml, etc.)
git add .
# Or add specific paths:
# git add gNNS_drone/ config/

# Commit
git commit -m "Add Jetson Nano scripts and config"

# Push to GitHub (origin)
git push origin main
```

Your config files (`config/mavlink_config.yaml`, `flight_config.yaml`, etc.) are **in the repo**, so they are pushed too. Only things in `.gitignore` (venv, `__pycache__`, logs) are not pushed.

## 2. Get the code on your laptop

**First time (clone):**
```bash
git clone https://github.com/SKULLNIN/GNNS_drone-updated2.git
cd GNNS_drone-updated2
```

**Later (already cloned, get latest):**
```bash
cd /path/to/GNNS_drone-updated2
git pull origin main
```

## 3. Remotes

- **origin** = GitHub (`https://github.com/SKULLNIN/GNNS_drone-updated2.git`) — use this to push from PC and pull on laptop.
- To add your **laptop as a second remote** (e.g. push directly to laptop over SSH):
  ```bash
  git remote add laptop ssh://user@LAPTOP_IP/path/to/repo.git
  git push laptop main
  ```
  (Only if your laptop has a Git server or bare repo set up.)

## 4. One-time setup on this PC (if needed)

```bash
git config user.name "Your Name"
git config user.email "your@email.com"
```

Then push with `git push origin main` (GitHub may ask for login or token).
