Prerequisites
=============

The following installation instructions are targeted towards Debian and Ubuntu Linux users. For other OS's, these installation instructions can serve as a guide, but will need to be adapted to suit your individual needs.

## Mining Robot Backend

Follow the build instructions in [README.txt](/README.txt) of this repo to make the backend server for Excahauler.

# PostgreSQL

Install the latest version of PostgreSQL, the database we use to record data for Excahauler.
Install the latest version of the C libraries for PostgreSQL.

```shell
sudo apt install postgresql postgresql-contrib libpq-dev libpq5
```

To have the robot send data to a database server machine, on the database server you'll need to:
 - Update /etc/postgresql/14/main/postgresql.conf so it has a line: ```listen_address = '*'```
 - Update /etc/postgresql/14/main/pg_hba.conf so it has a line like:
 ```host test_cpp all 10.10.10.0/24  md5```


## libpqxx

Follow the installation instructions in the [BUILD-CONFIGURE.md](https://github.com/jtv/libpqxx/blob/master/BUILDING-configure.md) `libpqxx` project file.

#### Tips for Installing libpqxx

```shell
cd ~/Downloads && git clone https://github.com/AuroraRoboticsLab/libpqxx.git && cd libpqxx
```

Configure and build

```shell
./configure
make
sudo make install
```
The library will be installed in the system directory. You may delete the `~/Downloads/libpqxx` folder, if you wish.


## Setup for PostgreSQL

At this time, the database is only a temporary test database. Create the temporary user password and database as follows.

Automated version:
``` shell
 ./make_database.sh 
```

#### Enter psql

``` shell
sudo -su postgres
psql
```

#### Change postgres User Password

```shell
# ALTER USER postgres PASSWORD 'asdf';
```

#### Create the Test Database

```shell
# CREATE DATABASE test_cpp;
```

Exit out of `psql` by repeatedly entering `exit` in the command until you return to the `lunacapture` directory.

## Execute lunacapture Makefile

```shell
make
```

## Activate the Backend Server

Open a separate terminal and activate the backend server in the [`/backend`](/backend) directory.

```shell
./backend --sim
```

## Activate lunacapture

In the lunacapture terminal window, activate `lunacapture`.

```shell
./lunacapture
```

You should see json values populating in the console. This data is automatically logged in the PostgreSQL database and is displayed here for visual purposes only.

Click on the display for `backend` and try moving the robot. As the robot moves, you should note that the `lunacapture` terminal json output updates.

When finished, hit `CTRL^C` to deactivate `lunacapture`.

## Find Data in the Database

The following is an example of how you can pull the `power_dump` values and their associated timestamps from the json database.

```shell
sudo -su postgres
psql
# \d test_conn;
# SELECT id,robot_json->'power_dump' as power, created_at FROM test_conn";
```



## Debug: Test libpq5 installation

Test the following command first to see if `libpq5` installed correctly.

```shell
apt-cache policy libpq5
```

If the value for `Candidate` is different than `Installed`, you may have come across an unusual issue that I experienced and had to solve as follows. 

First delete `libpq5` and `libpq-dev` from your system.

```shell
sudo apt remove --purge libpq5 libpq-dev
```

Discover what the latest version of libpq5 should be.

```shell
apt-cache policy libpq5
```

Look for the output value associated with the `Candidate` row.

Enter this value as follows during the installation. For example, where the candidate value is `14.7-0ubuntu0.22.04.1`:

```shell
sudo apt-get install libpq5=14.7-0ubuntu0.22.04.1 && sudo apt-get install libpq-dev
```

## Set Up Database Password


## Set Up Grafana

Go to [grafana.com/get](http://grafana.com/get).

Click "Download Grafana."

Download the version for Self-Managed and the chosen operating system.

Follow the terminal prerequisite installation process.

Perform the install using the `.deb` method using `wget`.

Run the `.deb` package.

Perform these steps to start Grafana:

```
sudo /bin/systemctl daemon-reload
sudo /bin/systemctl enable grafana-server
sudo /bin/systemctl start grafana-server
```

In your chosen web browser, enter `localhost:3000`.

The default username and password are both `admin`.

The system will force you to choose a new password.

## Set Up PostgreSQL Database Connection in Grafana

Click on the menu icon in the upper left corner.

Click `Connections`.

Click on `Data Sources`.

Search for `PostgreSQL` and initiate an instance.

Enter the default value for the address: `localhost:5432`.

Database is currently `test_cpp`.

User is `postgres`.

Password is the password from the `.dbpass` file.

Set the `TLS/SSL` mode to `disabled`.

You may choose to set `Max Lifetime` to `0` to represent infinity.

Click `Save & Test`. There should be a pop up that tells you if everything is correct.

#### (Optional) Lower the default Grafana panel refresh rate to 1 second

In the `/etc/grafana/grafana.ini` file search for `5s`.

Replace `5s` with `1s` or whatever value you prefer.

Return to the Grafana dashboard and refresh the browser page.

Create a new dashboard if you have not already.

In the dashboard page, click on the gear icon to go to the settings page.

Scroll to the dialogue box that lists possible auto refresh options.

Put in  `1s` as the minimum value in the list, and enter any other options preferred.

Save the settings.

Remember to save the dashboard itself by clicking the floppy disk icon.
