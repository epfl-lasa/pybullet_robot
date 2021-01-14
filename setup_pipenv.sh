#bin/sh
command -v python3 >/dev/null
if [ $? -ne 0 ];
then
  echo "Install python3 first!"
  exit 1
fi

export PATH="${HOME}/.local/bin:$PATH"
sudo apt install python3-pip
python3 -m pip install --user pipenv
pipenv install
pipenv shell