#!/bin/zsh
[[ ! -f /tmp/agent_node.pid ]] && supervisorctl restart all

