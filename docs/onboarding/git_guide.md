# Git Usage Guide

A comprehensive guide to using Git and contributing to projects. 
Please follow the [git setup](git_onboarding_instructions.md) guide first if you haven't already.

## Table of Contents

- [What is Git?](#what-is-git)
- [Understanding Git Concepts](#understanding-git-concepts)
- [Getting Started with a Project](#getting-started-with-a-project)
- [Basic Workflow](#basic-workflow)
- [Branching](#branching)
- [Working with Remote Repositories](#working-with-remote-repositories)
- [Fetching and Pulling](#fetching-and-pulling)
- [Committing and Pushing](#committing-and-pushing)
- [Advanced Topics](#advanced-topics)
- [Common Scenarios](#common-scenarios)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

## What is Git?

Git is a **version control system** - think of it as a sophisticated "undo" system for your code that:

  - Tracks every change you make to files
  - Lets multiple people work on the same project without conflicts
  - Keeps a complete history of your project
  - Allows you to create different versions (branches) of your project

**Key Terms:**

  - **Repository (repo):** A folder containing your project and its Git history
  - **Commit:** A snapshot of your changes with a descriptive message
  - **Branch:** A separate line of development (like a parallel universe of your code)
  - **Remote:** A version of your repository hosted online (like GitHub)

## Understanding Git Concepts

### The Three Areas

Git works with three main areas:

1. **Working Directory:** Your actual files that you can see and edit
2. **Staging Area (Index):** A prep area for changes you want to commit
3. **Repository:** Where Git stores the committed history

```
Working Directory → git add → Staging Area → git commit → Repository
```

### File States

Files in Git can be in different states:

  - **Untracked:** New files Git doesn't know about
  - **Tracked:** Files Git is monitoring
    - **Unmodified:** No changes since last commit
    - **Modified:** Changed but not staged
    - **Staged:** Ready to be committed

## Getting Started with a Project

Follow the [git setup](git_onboarding_instructions.md) to clone GNCSIM to your local machine. 

## Basic Workflow

**This is the standard workflow all team members should follow.** Each step uses Git commands that are explained in detail in the sections below:

```bash
# 1. Fetch latest changes from remote
git fetch origin

# 2. Check which branch you're currently on
git branch
# or for more detail:
git status

# 3. If on a feature branch, rebase on main to stay up-to-date
git checkout main
git pull origin main
git checkout your-feature-branch
git rebase main

# 4. Make changes to your files (edit, create, delete)

# 5. See what changed
git diff

# 6. Add changes to staging area
git add filename.txt           # Add specific file
git add .                      # Add all changes
git add *.js                   # Add all JavaScript files

# 7. Commit the changes
git commit -m "Describe what you changed"

# 8. Push to remote repository
git push
```

## Branching

Branches let you work on different features without affecting the main code.

### Creating and Managing Branches

```bash
# See all branches (* shows current branch)
git branch

# Create a new branch
git branch feature-branch

# Switch to the new branch
git checkout feature-branch

# Create and switch to a branch in one command
git checkout -b feature-branch
```

### Working with Branches

```bash
# Make changes in your branch
# Edit files, add, commit as usual

# Switch back to main branch
git checkout main

# See the difference between branches
git diff main feature-branch

# Delete a branch (only after it's merged)
git branch -d feature-branch

# Force delete a branch (be careful! You could lose all progress)
git branch -D feature-branch
```

## Working with Remote Repositories

### Tracking Remote Branches

```bash
# Push your branch and set it to track the remote
git push -u origin feature-branch

# See which branches are tracking remotes
git branch -vv
```

## Fetching and Pulling

### Understanding the Difference

- **Fetch:** Downloads changes but doesn't merge them
- **Pull:** Downloads changes AND merges them into your current branch

Assuming your branch is already tracking the remote (see above):
```bash
# Make sure you are on the branch
git checkout feature-branch

# Fetch all changes from remote (safe - doesn't change your code)
git fetch

# Merge all changes from remote into your branch:
git pull
```

To pull a branch that's on the remote (but not on your local computer yet)
```bash
# Fetch all remote branches
git fetch origin

# See all remote branches
git branch -r

# Checkout to the branch you want (git should set automatic tracking)
git checkout feature-branch

# Pull all changes (once in the branch)
git pull 
```

### When to Use Each

- Use `git fetch` when you want to see what others have done without changing your work
- Use `git pull` when you're ready to incorporate remote changes into your current branch

## Committing and Pushing

### Understanding the Difference

**Committing** and **pushing** are two separate operations in Git:

- **Commit:** Saves your changes to your local repository

  - Creates a permanent snapshot of your changes
  - Only affects your local copy of the project
  - Can be done offline
  - Multiple commits can be made before pushing

- **Push:** Uploads your commits to the remote repository

  - Shares your changes with others
  - Updates the remote repository (e.g., on GitHub)
  - Requires internet connection
  - Makes your work visible to teammates

You can make many commits locally before pushing them all at once to the remote repository.

### Making Good Commits

```bash
# Make sure you are in the right branch
git branch 

# Check which files are untracked, tracked, staged, and ready to commit 
git status

# Add specific files
git add file1.js file2.css

# Add all changes in current directory
git add .

# Add all changes in the entire repository
git add -A

# Commit with a message (recommended)
git commit -m "Add user login functionality"

# Add and commit in one step (only for tracked files)
git commit -am "Fix typo in README"
```

### Commit Message Best Practices

```bash
# Good commit messages are:
# - Present tense ("Add feature" not "Added feature")
# - Concise but descriptive
# - Start with a capital letter

# Examples of good commit messages:
git commit -m "Add user authentication"
git commit -m "Fix navigation menu on mobile"
git commit -m "Update README with installation instructions"
git commit -m "Remove deprecated API endpoints"
```

### Pushing Changes

Assuming your branch is already tracking remote:
```bash
# If branch is tracking remote:
git push
```

## Advanced Topics

### Merge Conflicts

Merge conflicts happen when two people change the same part of a file. Here's how to resolve them:

```bash
# When you encounter a conflict during merge/pull:
# Git will tell you which files have conflicts

**To resolve:**

1. Edit the file to choose which changes to keep
2. Remove the conflict markers (`<<<<<<<`, `=======`, `>>>>>>>`)
3. Add and commit the resolved file

```bash
# After editing the file:
git add conflicted-file.js
git commit -m "Resolve merge conflict in login function"
```

### Rebasing

Rebasing takes your commits and "replays" them on top of another branch, creating a cleaner, linear history.

**What rebasing does:**
- Takes your branch's commits and temporarily removes them
- Updates your branch to match the target branch (like main)
- Applies your commits one by one on top of the updated branch
- Rewrites commit history to make it appear as if your work was done on the latest version

**Visual example:**
```
Before rebase:           After rebase:
    A---B---C main           A---B---C main
        \                            \
         D---E feature                D'---E' feature
```

Your feature branch commits (D', E') are now based on the latest main commit (C) instead of the older commit (B). Main branch is unchanged - your commits are still only on your feature branch.

```bash
# Rebase your current branch onto main
git rebase main

# Interactive rebase (lets you edit commits)
git rebase -i HEAD~3  # Rebase last 3 commits

# Continue rebase after resolving conflicts
git rebase --continue

# Abort rebase if things go wrong
git rebase --abort
```

**Use rebasing to:**
- Keep a linear project history (no merge commits)
- Clean up your commits before sharing with others
- Update your feature branch with latest main branch changes
- Make it look like you started your work from the current state of main

**Important:** Never rebase commits that have been pushed and shared with others, as it rewrites history.

### Cherry Picking

Cherry picking lets you apply a specific commit from another branch:

```bash
# Find the commit hash you want
git log --oneline

# Apply that commit to current branch
git cherry-pick abc123

# Cherry pick multiple commits
git cherry-pick abc123 def456

# Cherry pick without committing (in case you want to modify)
git cherry-pick --no-commit abc123
```

### Stashing

Stashing temporarily saves your uncommitted changes:

```bash
# Stash your current changes
git stash

# Stash with a message
git stash save "Work in progress on login feature"

# See all stashes
git stash list

# Apply most recent stash
git stash pop

# Apply specific stash
git stash apply stash@{1}

# Delete a stash
git stash drop stash@{0}
```

## Common Scenarios

### Contributing to an Open Source Project

```bash
# 1. Create a feature branch (GNCSIM repo already cloned as per git onboarding instructions)
git checkout -b fix-typo

# 2. Make your changes and commit
git add .
git commit -m "Fix typo in documentation"

# 3. Push to the repository
git push origin fix-typo

# 4. Create a Pull Request on GitHub

# 5. Keep your branch updated
git checkout main
git pull origin main
```

### Syncing with Team Changes

```bash
# Daily workflow when working with a team:

# 1. Start your day by updating main
git checkout main
git pull origin main

# 2. Update your feature branch
git checkout your-feature-branch
git rebase main  # or git merge main

# 3. Work on your changes
# ... make changes ...
git add .
git commit -m "Your changes"

# 4. Push your work
git push origin your-feature-branch
```

### Undoing Changes

```bash
# Undo changes in working directory (before staging)
git checkout -- filename.txt
git restore filename.txt  # newer syntax

# Unstage files (after git add, before commit)
git reset HEAD filename.txt
git restore --staged filename.txt  # newer syntax

# Undo last commit (keep changes in working directory)
git reset --soft HEAD~1

# Undo last commit (discard changes completely)
git reset --hard HEAD~1

# Undo specific commit (creates new commit)
git revert abc123
```

## Best Practices

### Commit Practices

- **Commit often:** Small, frequent commits are better than large ones
- **Atomic commits:** Each commit should represent one logical change
- **Good messages:** Write clear, descriptive commit messages
- **Test before committing:** Make sure your code works before committing

### Branch Practices

- **Use descriptive branch names:** `feature/user-authentication` not `my-branch`
- **Keep branches focused:** One feature or fix per branch
- **Delete merged branches:** Clean up after merging
- **Stay updated:** Regularly sync with main branch

### Collaboration Practices

- **Pull before pushing:** Always get latest changes before pushing
- **Review before merging:** Use Pull Requests for code review
- **Communicate:** Use good commit messages and PR descriptions
- **Test integration:** Test your branch with latest main before merging

## Troubleshooting

### Common Issues and Solutions

**"Your branch is ahead of 'origin/main' by X commits"**
```bash
git push origin main
```

**"Your branch is behind 'origin/main' by X commits"**
```bash
git pull origin main
```

**"Merge conflict in file.txt"**
```bash
# Edit the file to resolve conflicts
git add file.txt
git commit -m "Resolve merge conflict"
```

**"Permission denied (publickey)"**
- Set up SSH keys with your Git hosting service
- Or use HTTPS URLs instead of SSH

**"Nothing to commit, working tree clean"**
- This is normal - means all your changes are committed

**Accidentally committed to wrong branch:**
```bash
# Move last commit to correct branch
git checkout correct-branch
git cherry-pick wrong-branch
git checkout wrong-branch
git reset --hard HEAD~1
```

### Getting Help

```bash
# Get help on any command
git help command-name
git command-name --help

# Quick help
git command-name -h

# See what Git version you have
git --version
```

### Useful Aliases

Speed up your workflow with aliases:

```bash
# Set up useful aliases
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.unstage 'reset HEAD --'
git config --global alias.last 'log -1 HEAD'
git config --global alias.visual '!gitk'

# Now you can use shortcuts:
git st      # instead of git status
git co main # instead of git checkout main
```

## Final Tips

1. **Practice in a test repository:** Create a dummy project to practice these commands
2. **Use a GUI if needed:** Tools like GitKraken, SourceTree, or VS Code's Git integration can help visualize what's happening
3. **Read the output:** Git gives helpful messages - read them!
4. **Don't panic:** Almost everything in Git can be undone
5. **Ask for help:** Your team members are usually happy to help with Git issues

Remember: Git has a learning curve, but once you understand the basic concepts, it becomes an incredibly powerful tool for managing your code and collaborating with others.

Happy coding! 🚀