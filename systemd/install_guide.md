# MAVROS Docker Compose as a systemd service (Jetson)

This guide shows how to run MAVROS (inside Docker Compose) automatically at boot on a Jetson, **and retry until `/dev/ttyTHS1` exists**.

---

## 1) Create the systemd service file

Create (or edit) this file:

```bash
sudo nano /etc/systemd/system/mavros-jetson.service
```

Paste this content:

```ini
[Unit]
Description=MAVROS (Docker Compose) - auto start + retry until UART present
After=docker.service network-online.target
Wants=network-online.target
Requires=docker.service

[Service]
Type=simple
WorkingDirectory=/home/zenith/aeac-2026


ExecStartPre=/bin/bash -lc 'until [ -c /dev/ttyTHS1 ]; do sleep 1; done'


ExecStart=/usr/bin/docker compose -p mavros-jetson -f compose/mavros.yml up --build
ExecStop=/usr/bin/docker compose -p mavros-jetson -f compose/mavros.yml down

Restart=always
RestartSec=2
StartLimitIntervalSec=0

[Install]
WantedBy=multi-user.target
```

Save and exit (Ctrl+O, Enter, Ctrl+X).

## 2) Reload systemd and enable at boot

Run:

```bash
sudo systemctl daemon-reload
sudo systemctl enable mavros-jetson
```

## 3) Start / restart the service

Start it now:

```bash
sudo systemctl start mavros-jetson
```

Or restart it (if it was already running):

```bash
sudo systemctl restart mavros-jetson
```

## 4) Check status and logs

Status:

```bash
systemctl status mavros-jetson --no-pager
```

Follow logs live:

```bash
journalctl -u mavros-jetson -f
```

## 5) Stop / disable (optional)

Stop:

```bash
sudo systemctl stop mavros-jetson
```

Disable auto-start at boot:

```bash
sudo systemctl disable mavros-jetson
```