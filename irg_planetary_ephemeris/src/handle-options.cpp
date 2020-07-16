// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "handle-options.h"

#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <list>

using namespace std;


static void
error_exit(char *message)
{
  cerr << "\nERROR in " << message << endl << endl;
  exit(1);
}

static void
system_error_exit(char *message)
{
  perror(message);
  exit(1);
}

static void 
usage(char *progName, Options &options, int numRequiredArgs,
      string argDescription)
{
  int numOptions = options.Length();
  if (options.UsageComment().empty())
  {
    string optionsPlaceholder = ((numOptions > 0) ? "[options] " : "");
    cerr << "\nUsage: " << progName  << " " << optionsPlaceholder << flush;
    if (argDescription.length() == 0)
      for (int i = 0; i < numRequiredArgs; ++i)
	cerr << "<Arg" << i << "> " << flush;
    else
      cerr << argDescription << flush;
    cerr << endl << endl;
  }
  else
  {
    cerr << "\nUsage: " << progName << " " << options.UsageComment()
	 << endl << endl;
  }

  if (numOptions > 0)
  {
    cerr << "options:\n" << endl;
    for (int i = 0; i < numOptions; i++)
    {
      Options::OptionMap::value_type option = options[i];
      cerr << "    -" << option.first << "\t" << option.second->Comment()
	   << endl;
    }
    cerr << endl;
  }

  exit(1);
}

static void
split_string(string const &input, const string &delimiters,
	     list<string> &strings, bool skip_white_space = true)
{
  const string::size_type length = input.length();
  string::size_type head = 0;
  string separators(delimiters);
  const string white_space(" \t\n");

  if (skip_white_space)
    separators += white_space;

  while (head < length)
  {
    // find first non-separator:
    head = input.find_first_not_of(separators, head);
    if (head == string::npos)		   // nothing left but separators
      return;

    // find the end of the token
    string::size_type tail = input.find_first_of(separators, head);

    // push token
    if (tail == string::npos)
    {
      strings.push_back(input.substr(head));
      return;
    }
    else
    {
      strings.push_back(input.substr(head, tail - head));
    }

    // setup indices for next token
    head = tail + 1;
  }
}

static string
trim_file_extension(const string &path)
{
  list<string> strings;
  string remainder;

  split_string(path, ".", strings);
  strings.pop_back();		// Get rid of the extension
  for (list<string>::iterator element = strings.begin();
       element != strings.end(); ++element)
    remainder += *element;

  return remainder;
}

int
OptionInfo::NumStringValues(const string &valuesString)
{
  list<string> stringList;
  split_string(valuesString, string(" ,"), stringList);
  return stringList.size();
}
const void
OptionInfo::Value(string &stringValue) const
{
  if (m_numValues == 0)
    return;

  list<string> stringList;
  split_string(m_valuesString, string(" ,"), stringList);

  stringValue = *stringList.begin();
}

const void
OptionInfo::Value(int &intValue) const
{
  if (m_numValues == 0)
    return;

  string stringValue;
  // We first extract the first string value from m_valuesString...
  Value(stringValue);
  if (stringValue.length() == 0)
    return;

  intValue = strtol(stringValue.c_str(), 0, 10);
}

const void
OptionInfo::Value(double &doubleValue) const
{
  if (m_numValues == 0)
    return;

  string stringValue;
  Value(stringValue);
  if (stringValue.length() == 0)
    return;

  doubleValue = strtod(stringValue.c_str(), 0);
}

const void
OptionInfo::Values(vector<string> &stringValues) const
{
  list<string> stringList;
  split_string(m_valuesString, string(" ,"), stringList);

  stringValues.resize(stringList.size());
  int i = 0;
  for (list<string>::const_iterator element = stringList.begin();
       element != stringList.end(); ++element)
  {
    stringValues[i] = *element;
    ++i;
  }
}

const void
OptionInfo::Values(vector<int> &intValues) const
{
  vector<string> stringValues;
  // We first extract a vector of string values from m_valuesString...
  Values(stringValues);

  assert(stringValues.size() == m_numValues);

  intValues.resize(m_numValues);
  for (int i = 0; i < m_numValues; i++)
    intValues[i] = strtol(stringValues[i].c_str(), 0, 10);
}

const void
OptionInfo::Values(vector<double> &doubleValues) const
{
  vector<string> stringValues;
  Values(stringValues);

  assert(stringValues.size() == m_numValues);

  doubleValues.resize(m_numValues);
  for (int i = 0; i < m_numValues; i++)
    doubleValues[i] = strtod(stringValues[i].c_str(), 0);
}

int 
HandleOptions(int argc, char *argv[], Options &options, int numRequiredArgs,
	      string argDescription)
{
  extern char *optarg;
  extern int optind;
  string stringArg;
  int ch;

  const string optstring = options.GetOptString().c_str();
  opterr = 0;			// We'll handle the error messages...

  while((ch = getopt(argc, argv, optstring.c_str())) != EOF)
  {
    if (ch == '?')		// ? indicates unknown option
    {
      cerr << endl;
      // optopt contains the actual char
      cerr << argv[0] << ": "
	   << "unknown option -" << char(optopt) << endl;
      usage(argv[0], options, numRequiredArgs, argDescription);
    }

    OptionInfo *optionInfo = 0;

    if ((optionInfo = options.GetOptionInfo(ch)) != 0)
    {
      int numValues = optionInfo->NumValues();
      if (numValues > 0)
      {
	if (optarg == 0)
	{
	  cerr << endl;
	  cerr << argv[0] << ": "
	       << "optarg is 0 for -" << char(ch) << " option" << endl;
	  usage(argv[0], options, numRequiredArgs, argDescription);
	}
	if (optionInfo->SetValuesString(optarg) != numValues)
	{
	  cerr << endl;
	  cerr << argv[0] << ": "
	       << "couldn't parse option -" << char(ch) << " argument" << endl;
	  usage(argv[0], options, numRequiredArgs, argDescription);
	}
      }
      else
      {
	optionInfo->SetValuesString("");
      }
    }
    else
    {
      usage(argv[0], options, numRequiredArgs, argDescription);
    }
  }
  
  int numArgs = argc - optind;
  if (numArgs < numRequiredArgs)
  {
    cerr << endl;
    cerr << argv[0] << ": "
	 << numRequiredArgs << " arguments required, " << numArgs << " given."
	 << endl;
    usage(argv[0], options, numRequiredArgs, argDescription);
  }

  return optind;
}

void
Usage(char *argv[], Options &options, int numRequiredArgs,
      string argDescription)
{
  usage(argv[0], options, numRequiredArgs, argDescription);
  exit(-1);
}
