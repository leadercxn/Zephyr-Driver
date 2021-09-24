#
# Copyright (c) 2021 Jackychen
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADS1115 ads1115.c)
