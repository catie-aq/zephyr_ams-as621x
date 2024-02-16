# Copyright (c) 2024 CATIE
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(as621x.c)
zephyr_library_sources_ifdef(CONFIG_AS621X_TRIGGER as621x_trigger.c)
