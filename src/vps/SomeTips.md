# Some tips for using Linux

## Virtual Environment

```
#Install pew, https://pypi.org/project/pew
pip install pew

#After install, you need to check whether following 'source ~~' command line is in your ~/.bashrc
#otherwise, you can add by yourself manually
source $(pew shell_config)


#You neet to following alias at your ~/.bashrc
alias pewnew2='pew new $1 -p /usr/bin/python2.7'
alias pewnew3='pew new $1 -p /usr/bin/python3'


#You can make new virtual environment with anyname as follows:
pewnew3 3venv3.5

#You can enter that virtual envrionement using this:
pew workon 3venv3.5

#After enter virtual environment, you enjoy to install proper pip package and run .py code in there
#If you want to exit virtual envirionmenet, quit terminal or Ctrl+D

```

## Debugging python code in terminal
```
#Install ipdb
pip install ipdb

#Add 1 line at the begenning of a foo.py file you want to debug as follows:
from ipdb import set_trace as bp

#Then you can set breakpoint any line you want to stop using bp()
#foo.py
print(~~~)
bp()
some codes~~~

#You run python code, then you can stop the code line right befor bp()
python foo.py

#You can debug with pdb command
?
help
!someVariable
n
c
...
```


## Git in Linux with terminal
```
# You need to add following alias in your ~/.bashrc for conveience.
# You need to change
#        YourGitID to your ID
#        YourMailAddress to your mail
#        XXXXXXXXXXXX to your password of github

alias gitcloneDeepGuider='git clone https://github.com/deepguider/DeepGuider;cd DeepGuider;git config user.name YourGitID;git config user.email YourMailAddress'
alias gitconfiguser='git config user.name YourGitID;git config user.email YourMailAddress'
alias gitpushDeepGuider='git config push.default matching;git commit -a;git pull;git push https://YourGitID:XXXXXXXXXXXXX@github.com/deepguider/DeepGuider'

# You can git clone:
gitcloneDeepGuider

# After some modification, you can push :
gitpushDeepGuider
```


Written by Seungmin Choi, ccsmm@etri.re.kr
