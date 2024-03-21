#!/bin/bash

# Shell script to make sure SSL and Network are ready before running fwdip.py

# Function to check if SSL certificates are set up
check_ssl_certificates() {
    # Check if SSL certificates directory exists and is not empty
    if [ -d "/etc/ssl/certs" ] && [ "$(ls -A /etc/ssl/certs)" ]; then
        return 0  # SSL certificates are set up
    else
        return 1  # SSL certificates are not set up
    fi
}

# Wait for network connectivity
while ! ping -c 1 google.com >/dev/null 2>&1; do
    sleep 1
done

# Wait for SSL certificates to be set up
while ! check_ssl_certificates; do
    sleep 1
done

/usr/bin/python3 ./fwdip.py
