# Set custom build config file.
setconfig() {
	if [ ! -d ~/.gyp ] ; then
		echo creating .gyp dir
		mkdir ~/.gyp
	else
		echo ~/.gyp already exists
	fi

cat > ~/.gyp/include.gypi << EOF
{
     'variables': {
	'build_svmp_video_capture': $1,
     },
}
EOF
}


