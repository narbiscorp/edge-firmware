# Standalone narbis_protocol component (vendored copy of the
# protocol/narbis_protocol.{h,c} from narbiscorp/narbis-earclip).
# Re-sync this folder if the upstream protocol changes (config_version
# bump, new UUIDs, new enums).
idf_component_register(
    SRCS "narbis_protocol.c"
    INCLUDE_DIRS "."
)
