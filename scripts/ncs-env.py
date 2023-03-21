#!/usr/bin/env python3

# In order to set up the environment for building NCS based projects,
# Nordic recommends starting bash from the Toolchain Manager GUI app.
#
# This script parses environment.json and programmatically sets up
# the appropriate variables, so that the process can be automated.
# It searches the default NCS installation paths for Linux and macOS.
#
# If $NCS_PARENT is set, it will check there too.  The $NCS_PARENT
# directory should contain "toolchains" and "vX.Y.Z" subdirectories.
#
# See also:
# https://devzone.nordicsemi.com/f/nordic-q-a/91499/vscode-replicating-build-configurations-preset-bugs-command-line/384915

import json, os, sys

def toolchain_var_list(path, ncs_vers):
    toolchain_base = "%s/toolchains/v%s" % (path, ncs_vers)
    toolchain_json = "%s/environment.json" % (toolchain_base)

    # a valid NCS installation has one directory under toolchains/
    # and another directory containing Zephyr
    try:
        os.stat("%s/v%s/zephyr/zephyr-env.sh" % (path, ncs_vers))
        os.stat(toolchain_json)
    except:
        return None

    with open(toolchain_json, "r") as f:
        j = json.load(f)

    var_dict = dict()
    for v in j["env_vars"]:
        k = v["key"]

        if v["type"] != "relative_paths":
            print("warning: skipping var '%s' of unknown type '%s'" %
                  (k, v["type"]))
            continue

        output_value = ""
        if v["existing_value_treatment"] == "overwrite":
            pass
        elif v["existing_value_treatment"] == "prepend_to":
            if k in os.environ:
                output_value = os.environ[k]
        else:
            print("warning: var '%s' has unknown existing_value_treatment '%s'" %
                (k, v["existing_value_treatment"]))

        # since this loop prepends each value, the first item in v["values"]
        # winds up at the end of the string
        v["values"].reverse()

        for value in v["values"]:
            value = toolchain_base + "/" + value
            if output_value == "":
                output_value = value
            else:
                output_value = value + ":" + output_value
        var_dict[k] = output_value

    # rather then sourcing zephyr-env.sh (which almost never changes),
    # just hardcode the values here
    z_base = "%s/v%s/zephyr" % (path, ncs_vers)
    var_dict["ZEPHYR_BASE"] = z_base
    var_dict["PATH"] = "%s/scripts:%s" % (z_base, var_dict["PATH"])

    return var_dict

def exec_with_vars(var_dict, cmdline):
    for k in var_dict.keys():
        os.environ[k] = var_dict[k]
    # never returns
    os.execvp(cmdline[0], cmdline)

    sys.stderr.write("Could not exec %s" % cmdline[0])
    sys.exit(1)

def main():
    ncs_vers = sys.argv[1]

    paths = list()
    if "NCS_PARENT" in os.environ:
        paths.append(os.environ["NCS_PARENT"])
    paths.append(os.environ["HOME"] + "/ncs")
    paths.append("/opt/nordic/ncs")

    for p in paths:
        var_dict = toolchain_var_list(p, ncs_vers)
        if var_dict != None:
            exec_with_vars(var_dict, sys.argv[2:])

    sys.stderr.write("Could not find NCS v%s in:\n" % ncs_vers)
    for p in paths:
        sys.stderr.write("    %s\n" % p)
    sys.stderr.write("\n")
    sys.exit(1)

if __name__ == "__main__":
    main()
