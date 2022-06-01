#!/bin/bash
exec /usr/bin/tini -- /usr/bin/supervisord -n
