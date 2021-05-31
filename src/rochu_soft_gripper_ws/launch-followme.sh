#!/bin/bash

# Generate 8 panes in tmux session
tmux split-window -h
tmux split-window -v 
tmux split-window -v 
tmux select-pane -t left
tmux select-pane -t right
tmux split-window -v 
tmux select-pane -t left
tmux split-window -v 
tmux split-window -v 
tmux select-pane -t 0
tmux split-window -v 
