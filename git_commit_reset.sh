#!/bin/bash
git filter-branch --env-filter '
OLD_EMAIL="nie_bayes@gmail.com"
OLD_EMAIL2="870530032@qq.com"
OLD_EMAIL3="lzx@nie-bayes.local"
OLD_NAME="Nie Shicong"
OLD_NAME2="nie_bayes"
OLD_NAME3="Bayes Nie"
OLD_NAME4="lzx"
CORRECT_NAME="niebayes"
CORRECT_EMAIL="niebayes@gmail.com"
if [ "$GIT_COMMITTER_EMAIL" = "$OLD_EMAIL" ] || [ "$GIT_COMMITTER_NAME" = "$OLD_NAME" ] || [ "GIT_COMMITTER_NAME" = "$OLD_NAME2" ] || [ "GIT_COMMITTER_NAME" = "$OLD_NAME3" ] || [ "GIT_COMMITTER_EMAIL" = "$OLD_EMAIL2" ] || [ "GIT_COMMITTER_NAME" = "$OLD_NAME4" ] || [ "$GIT_COMMITTER_EMAIL" = "$OLD_EMAIL3" ]
then
export GIT_COMMITTER_NAME="$CORRECT_NAME"
export GIT_COMMITTER_EMAIL="$CORRECT_EMAIL"
fi
if [ "$GIT_AUTHOR_EMAIL" = "$OLD_EMAIL" ] || [ "$GIT_AUTHOR_NAME" = "$OLD_NAME" ] || [ "GIT_AUTHOR_NAME" = "$OLD_NAME2" ] || [ "GIT_AUTHOR_NAME" = "$OLD_NAME3" ] || [ "GIT_AUTHOR_EMAIL" = "$OLD_EMAIL2" ] || [ "GIT_AUTHOR_NAME" = "$OLD_NAME4" ] || [ "$GIT_AUTHOR_EMAIL" = "$OLD_EMAIL3" ]
then
export GIT_AUTHOR_NAME="$CORRECT_NAME"
export GIT_AUTHOR_EMAIL="$CORRECT_EMAIL"
fi' -f --tag-name-filter cat -- --branches --tags

# Usage 
# sh git_commit_reset.sh 
# git push --force --tags origin 'refs/heads/*'