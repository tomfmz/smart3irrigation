# smart3irrigation

## Software Setup
* install [vsCode](https://code.visualstudio.com/)
* install [Git](https://git-scm.com/download/win)
* create [GitHub Acc](https://github.com/)

## Workspace Setup
* open vsCode and install following extensions:
    * Git Graph
    * Git Lens
* create a local workspace folder
* open workspace folder with vsCode under File-> Open Folder
* open the vsCode Terminal under View->Terminal


The code can be cloned from GitHub into a local workspace 

```bash
cd ../<YOUR_WORKSPACE>
git clone https://github.com/tomfmz/smart3irrigation.git
```


## Git workflow
If you are using this library for development, please follow the proper procedure for creating a new development branch.
### Create your new branch
Checking out on to the develop banch and create your new feature branch from there.

Checkout to develop branch
```bash
git checkout develop
```
Create your new branch
```bash
git checkout -b <YOUR_BRANCH>
```
Check to see if branch was created and is active
```bash
git branch
```
If your branch name appears in the list, then everything was successful. You have now created a branch off of the .

**NOTE: DO NOT PUSH TO THE MASTER BRANCH UNLESS YOU KNOW WHAT YOU'RE DOING!!!** Always push to your development branch first. A merge can be done by the maintainer at a later date.

### Version Control

If you want to commit (save changes locally) and push your local changes to the origin (git repository).

```bash
cd ../<YOUR_WORKSPACE>
git status
git add <YOUR_NEWorMODIFIED_FILES>
git commit -m "<YOUR_COMMIT_MESSAGE>"
git status
git push origin <YOUR_BRANCH>
```
Update your local repository

```bash
git fetch
```
check if your current branch ist up to date
```bash
git status
```
if not use git pull to update 
```bash
git pull
```