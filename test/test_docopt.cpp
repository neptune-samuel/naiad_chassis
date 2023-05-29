
#include <docopt/docopt.h>

#include <iostream>

// static const char USAGE[] =
// R"(
// usage: git add [options] [--] [<filepattern>...]

// options:
//     -h, --help
//     -n, --dry-run        dry run
//     -v, --verbose        be verbose
//     -i, --interactive    interactive picking
//     -p, --patch          select hunks interactively
//     -e, --edit           edit current diff and apply
//     -f, --force          allow adding otherwise ignored files
//     -u, --update         update tracked files
//     -N, --intent-to-add  record only the fact that the path will be added later
//     -A, --all            add all, noticing removal of tracked files
//     --refresh            don't add, only refresh the index
//     --ignore-errors      just skip files which cannot be added because of errors
//     --ignore-missing     check if - even missing - files are ignored in dry run
// )";

static const char USAGE[] =
R"(Naval Fate.

    Usage:
      naval_fate ship new <name>...
      naval_fate ship <name> move <x> <y> [--speed=<kn>]
      naval_fate ship shoot <x> <y>
      naval_fate mine (set|remove) <x> <y> [--moored | --drifting]
      naval_fate (-h | --help)
      naval_fate --version

    Options:
      -h --help     Show this screen.
      --version     Show version.
      --speed=<kn>  Speed in knots [default: 10].
      --moored      Moored (anchored) mine.
      --drifting    Drifting mine.
)";

int main(int argc, const char** argv)
{
    std::map<std::string, docopt::value> args
        = docopt::docopt(USAGE,
                         { argv + 1, argv + argc },
                         true,               // show help if requested
                         "Naval Fate 2.0");  // version string

    for(auto const& arg : args) {
        std::cout << arg.first << " <-> " << arg.second << std::endl;
    }

    return 0;
}
