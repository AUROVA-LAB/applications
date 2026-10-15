#!/bin/bash
set -e

DEFAULT_USER=${DEFAULT_USER:-'ubuntu'}
DEFAULT_USER_UID=${USER_UID:-'1000'}
DEFAULT_USER_GID=${USER_GID:-'1000'}
SHELL=${DEFAULT_SHELL:-$SHELL}

if [[ $(id -u) -eq 0 ]]; then
	echo "Set disable_coredump false" >> /etc/sudo.conf
else
	echo "Set disable_coredump false" | sudo tee -a /etc/sudo.conf > /dev/null
fi

# override if $USER env exists
if [[ ! -z "$USER" ]]; then
	DEFAULT_USER=$USER
fi

# create development user
if [[ $(id -u) -eq 0 ]]; then
	RUN_AS=(runuser --user "${DEFAULT_USER}" --)

	# if the user does not exist, create user
	if [[ $(id -u ${DEFAULT_USER} 2> /dev/null) != ${DEFAULT_USER_UID} ]]; then
		echo creating user ${DEFAULT_USER}
		groupadd -g "${DEFAULT_USER_GID}" "${DEFAULT_USER}"
		useradd --create-home --home-dir /home/${DEFAULT_USER} --uid ${DEFAULT_USER_UID} --shell /bin/bash \
		    --gid ${DEFAULT_USER_GID} --groups sudo ${DEFAULT_USER}
		echo "${DEFAULT_USER}:${DEFAULT_USER}" | chpasswd && \
		echo "${DEFAULT_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
		touch /home/${DEFAULT_USER}/.sudo_as_admin_successful
	else
		echo "${DEFAULT_USER}:${DEFAULT_USER}" | chpasswd
	fi

	# setup ros environment
	mkdir -p /home/${DEFAULT_USER}/ros2_ws/src \
	echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/${DEFAULT_USER}/.bashrc

	chown -R ${DEFAULT_USER}:${DEFAULT_USER} /home/${DEFAULT_USER}

	# mount develop workspace
	if [ -e /ws ]; then
	ln -s /ws /home/${DEFAULT_USER}/ros2_ws/src
	fi

	DEFAULT_USER_UID="$("${RUN_AS[@]}" id -u)"
	DEFAULT_USER_GID="$("${RUN_AS[@]}" id -g)"
	HOME="/home/${DEFAULT_USER}"
else # use existing user
	RUN_AS=()
	DEFAULT_USER="$(whoami)"
	DEFAULT_USER_UID="$(id -u)"
	DEFAULT_USER_GID="$(id -g)"
	if [[ ! -e /home/${DEFAULT_USER}/.bashrc ]]; then
		cp /etc/skel/.* /home/$DEFAULT_USER/
	fi
fi

echo "Launched container with user: $DEFAULT_USER, uid: $DEFAULT_USER_UID, gid: $DEFAULT_USER_GID"

cd $HOME

if which "$1" > /dev/null 2>&1 ; then
	source /opt/ros/${ROS_DISTRO}/setup.bash
	exec "${RUN_AS[@]}" "$@"
else
	echo "$@" | "${RUN_AS[@]}" "$SHELL" -li
fi