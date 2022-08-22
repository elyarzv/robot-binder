This workspace is meant to be used to cache all packages that are not being
actively develop in the current project.

The rule of thumb is that if you need to clone a repository from github/gitlab
etc it goes in the underlay_ws.
If the package is developed in AIS but not actively developed, packages like
ais_msgs, cp_msgs, etc those packages go here.

Please use apt to install any pacakge that is needed trough package.xml or
apt.pkg, clone a repo that is publicly avalilable in the underlay_ws if it is
absolutley necessary.