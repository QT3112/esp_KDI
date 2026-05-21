idf_component_register(SRCS "web_rc.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_server esp_timer drone_config flight_control battery math_utils
                    EMBED_TXTFILES "web_rc.html")
