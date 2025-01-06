
#echo ""
ZSH_THEME="apple"
plugins=(git)
source $HOME/.oh-my-zsh/oh-my-zsh.sh

export ZSH=$HOME/.oh-my-zsh


lfcd () {
        tmp="$(mktemp)"
        ~/lf-hp -last-dir-path="$tmp" "$@"
        if [ -f "$tmp" ]
        then
                dir="$(cat "$tmp")"
                rm -f "$tmp" > /dev/null
                [ -d "$dir" ] && [ "$dir" != "$(pwd)" ] && cd "$dir"
        fi
}
bindkey -s '^[E' 'lfcd\n'

alias .="cd .."            # Up one level
alias ..="cd ../.."        # Up two levels
alias ...="cd ../../.."    # Up three levels
alias c="clear"
alias df='df -H'
alias du='du -ch'
# alias lu="du -ha --max-depth=1"
alias lu="du -ha --max-depth=1 | sort --human-numeric-sort "
# alias pwdc="pwd | xclip -selection clipboard"
alias history="fc -l 1"
alias o="xdg-open"
alias mv="mv -iv" #  if duplicate or exist must ask first
alias cp="cp -ivr" #  if duplicate or exist must ask first

alias v="vim"
alias b="cd -"              # Return to previous directory
alias d="cd ~/.dotfiles"
alias h="cd ~"              # Home directory
alias l="ls -Fh --color --group-directories-first"
alias la="ls -AFh --color --group-directories-first"
alias ll="ls -AFhlo --color --group-directories-first"
alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'
# alias termuxsshmachineme='ssh metfoil@192.168.1.5'

function termuxsshmachineme(){
#ssh -p 8022 u0_a359@192.168.1.23
export DISPLAY=:0
ssh metfoil@192.168.1.24
retVal=$?
if [ $retVal -ne 0 ]; then
    echo "Error"
    printf "%s" "192.168.1. ?(input|1-24?):"
    read -r inputip
    printf "%s" "u0_a359? username?@192.168.1.${inputip}(input username):"
    read -r inputname
    [[ -z "$inputname" ]] && inputname="metfoil"
    ssh -p 8022 "${inputname}@192.168.1.${inputip}"
fi
 
}

alias p="termuxsshmachineme"

if pgrep -x "sshd" > /dev/null; then
	echo "sshd is running"
else
	echo "sshd is not running"
	setsid -f "sshd" >/dev/null 2>&1
	sshd &
	echo "running sshd"
fi
