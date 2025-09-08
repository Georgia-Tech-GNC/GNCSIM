# Git Installation and Setup

## Step 1: Install Git

### Windows
1. Download Git from [git-scm.com](https://git-scm.com/download/win)
2. Run the installer with default settings
3. Open Command Prompt or PowerShell to verify installation:
   ```
   git --version
   ```

### Mac
**Option 1 - Using Homebrew (recommended):**
1. Install Homebrew if you don't have it: [brew.sh](https://brew.sh)
2. Run in Terminal:
   ```
   brew install git
   ```

**Option 2 - Download installer:**
1. Download from [git-scm.com](https://git-scm.com/download/mac)
2. Run the installer

### Linux (Ubuntu/Debian)
```bash
sudo apt update
sudo apt install git
```

### Linux (CentOS/RHEL/Fedora)
```bash
sudo yum install git
# or for newer versions:
sudo dnf install git
```

## Step 2: Configure Git (First Time Only)

Set up your identity for commits:
```bash
git config --global user.name "Your Full Name"
git config --global user.email "your.email@example.com"
```

## Step 3: Clone the Repository

1. Navigate to your desired project directory:
   ```bash
   cd ~/Documents  # or wherever you want the project
   ```

2. Clone the repository:
   ```bash
   git clone https://github.gatech.edu/gnc/GNCSIM.git
   ```

3. Navigate into the project directory:
   ```bash
   cd GNCSIM
   ```

## Step 4: Verify Setup

Check that everything is working:
```bash
git status
```

You should see a message about being on the main/master branch with a clean working tree.

## Troubleshooting

**"git command not found"**
- Restart your terminal/command prompt after installation
- On Windows, make sure Git was added to your PATH during installation

**Permission denied when cloning**
- Make sure you have access to the repository

**Need help?**
- Reach out to the controls lead
- Check with a team member
- Review Git documentation at [git-scm.com/doc](https://git-scm.com/doc)