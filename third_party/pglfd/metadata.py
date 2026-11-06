from conan import ConanFile


class PglfdMetaConan(ConanFile):
    # ######################################################### #
    #                package meta-information
    # ######################################################### #

    # * author (string) [required]
    #       package provider (and maintainer)
    author = "Maximilian Mühlbauer <maximilian.muehlbauer@dlr.de>"

    # * developers (list) [optional]
    #       additional list of package developers, excluding the 'author'
    #       e.g. ["Michaela Musterfrau <Michaela.Musterfrau@dlr.de>", "Max Mustermann <Max.Mustermann@dlr.de>"]
    developers = []

    # * license (string) [recommended]:
    #       please use the "official" licence identifier
    #       you can lookup the identifier on https://spdx.org/licenses/
    license = "proprietary"

    # * url (string) [required]
    #       url to internal rmc-github repository
    url = "https://rmc-github.robotic.dlr.de/tplab/pglfd"

    # * homepage (string) [recommended]
    #       url to project page for containing more documentation
    homepage = ""

    # * package defaults (string)
    #       used when no user/channel is specified, e.g. 'conan info .'
    #
    default_user = "tplab"
    default_channel = "snapshot"

    # * exports (array of files/pattern) [optional]:
    #       defines all files that are part of the recipe
    exports = ["metadata.py", "seqm.yaml"]
