# RQt robot monitor

rqt_robot_monitor displays diagnostics_agg topics messages that are published by diagnostic_aggregator. rqt_robot_monitor is a direct port to rqt of robot_monitor. All diagnostics are fall into one of three tree panes depending on the status of diagnostics (normal, warning, error/stale). Status are shown in trees to represent their hierarchy. Worse status dominates the higher level status.

    Ex. 'Computer' category has 3 sub devices. 2 are green but 1 is error. Then 'Computer' becomes error. 

You can look at the detail of each status by double-clicking the tree nodes.
Currently re-usable API to other pkgs are not explicitly provided.