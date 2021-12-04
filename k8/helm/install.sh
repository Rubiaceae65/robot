#!/usr/bin/env bash
. ~/.profile
helm install simulation . -n simul
helm upgrade simulation . -n simul
