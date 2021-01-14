#bin/sh
command -v python3 >/dev/null
if [ $? -ne 0 ];
then
  echo "Install python3 first!"
else
  python3 -m pip install --user virtualenv
  python3 -m venv venv
  source venv/bin/activate
  pip3 install -r requirements.txt
  pip3 install src/pybullet_robots
  pip3 install src/pybullet_controllers
  pip3 install src/pybullet_simulation
fi