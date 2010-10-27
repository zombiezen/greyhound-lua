#!/bin/bash

clean_tempdir()
{
    if [[ ! -z "$TEMPDIR" && -e "$TEMPDIR" ]]
        then rm -rf "$TEMPDIR"
    fi
}

crash()
{
    echo "$1" 1>&2
    clean_tempdir
    exit 1
}

validFormat()
{
    if [[ "$1" == "tar" || "$1" == "tar.bz2" || "$1" == "tar.gz" || "$1" == "zip" ]]
        then return 0
    fi
    
    return 1
}

ARFMT=${1-"zip"}
validFormat "$ARFMT"
isValid=$?

if [[ $# -gt 1 || isValid -ne 0 ]]
then
    echo "usage: build-archive.sh FORMAT"
    echo
    echo "where FORMAT is one of:"
    echo "- tar"
    echo "- tar.bz2"
    echo "- tar.gz"
    echo "- zip"
    exit 1
fi

REVNO="$(bzr revno)"
DESTROOT="GreyhoundLua"
DESTAR="GreyhoundLua-r${REVNO}.${ARFMT}"

TEMPDIR="`mktemp -d -t GreyhoundLua-build.XXXX`" || crash "** Couldn't create temporary directory"
bzr export --format=dir "$TEMPDIR/$DESTROOT" || crash "** bzr could not export"


pushd "$TEMPDIR" > /dev/null
# Build support files
swig -lua -c++ "$DESTROOT/wpilib.i" || crash "** SWIG failed"
# Archive everything
case "$ARFMT" in
    "zip" ) zip --no-wild --quiet -rmT "$DESTAR" "$DESTROOT" || crash "** zip failed";;
    "tar" ) tar -cf "$DESTAR" "$DESTROOT" || crash "** tar failed";;
    "tar.bz2" ) tar --bzip2 -cf "$DESTAR" "$DESTROOT" || crash "** tar failed";;
    "tar.gz" ) tar --gzip -cf "$DESTAR" "$DESTROOT" || crash "** tar failed";;
    * )
        crash "** Invalid zip format"
    ;;
esac
popd > /dev/null

# Move directory back
mv "$TEMPDIR/$DESTAR" .

# Clean up
clean_tempdir
exit 0
