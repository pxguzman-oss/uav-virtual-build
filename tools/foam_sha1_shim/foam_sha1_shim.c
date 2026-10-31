/*
 * Workaround for the Ubuntu 24.04 'openfoam' package (OpenFOAM.com v1912, built with GCC 13):
 * Foam::dictionary::digest() streams the dictionary into an OSHA1stream whose stream state goes bad
 * ("FOAM FATAL IO ERROR: error in IOstream "sha1" for operation operator<<(Ostream&, const word&)"),
 * which aborts EVERY run that has a 'functions' entry (forceCoeffs, yPlus, ...).
 * The digest is only used by functionObjectList to detect edits of the dictionary at run time, so we
 * interpose a version that returns a constant (all-zero) 20-byte SHA1Digest.
 * Itanium C++ ABI: SHA1Digest (20 bytes) is returned through a hidden pointer (rdi), 'this' in rsi.
 * Build: gcc -O2 -shared -fPIC -o libfoam_sha1_shim.so foam_sha1_shim.c
 * Use:   LD_PRELOAD=/path/libfoam_sha1_shim.so simpleFoam ...
 */
#include <string.h>
void *_ZNK4Foam10dictionary6digestEv(void *ret, const void *self)
{
    (void)self;
    memset(ret, 0, 20);
    return ret;
}
