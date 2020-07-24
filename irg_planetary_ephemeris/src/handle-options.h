// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef _HANDLE_OPTIONS_H_
#define _HANDLE_OPTIONS_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>

class OptionInfo
{
public:
  OptionInfo(std::string comment, std::string valueType, int numValues = 0) :
    m_isSet(false),
    m_comment(comment),
    m_valueType(valueType),
    m_numValues(numValues) { }

  const bool IsSet() const { return m_isSet; }

  const std::string &Comment() const { return m_comment; }
  std::string &Comment() { return m_comment; }

  const std::string &ValueType() const { return m_valueType; }
  std::string &ValueType() { return m_valueType; }

  const int &NumValues() const { return m_numValues; }
  int &NumValues() { return m_numValues; }

  const std::string &GetValuesString() const
  {
    return m_valuesString;
  }
  int SetValuesString(const std::string &valuesString)
  {
    m_isSet = true;
    int numValues = OptionInfo::NumStringValues(valuesString);
    m_valuesString = valuesString;
    return numValues;
  }

  const void Value(std::string &stringValue) const;
  const void Value(int &intValue) const;
  const void Value(double &doubleValue) const;
  const void Values(std::vector<std::string> &stringValues) const;
  const void Values(std::vector<int> &intValues) const;
  const void Values(std::vector<double> &intValues) const;
  
private:
  static int NumStringValues(const std::string &valuesString);

  bool m_isSet;
  std::string m_comment;
  std::string m_valueType;
  int m_numValues;
  std::string m_valuesString;
};

class Options
{
public:
  typedef std::map<const char, OptionInfo *> OptionMap;

  Options() {}
  Options(std::string usageComment) : m_usageComment(usageComment) {}

  std::string &UsageComment() { return m_usageComment; }

  Options::OptionMap::value_type operator[](int i)
  {
    Options::OptionMap::iterator element = m_options.begin();

    for (int count = 0; element != m_options.end(); count++, element++)
      if (count == i)
	return *element;

    return *element;
  }
  int Length() { return m_options.size(); }

  const std::string GetOptString() const
  {
    std::string getOptString;

    for (Options::OptionMap::const_iterator element = m_options.begin();
	 element != m_options.end(); element++)
    {
      getOptString += element->first;
      if (element->second->NumValues() > 0)
	getOptString += ":";
    }

    return getOptString;
  }

  void Add(const char flag, std::string comment,
	   std::string valueType, int numValues = 0)
  {
    OptionInfo *optionInfo = new OptionInfo(comment, valueType, numValues);
    m_options.insert(OptionMap::value_type(flag, optionInfo));
  }

  void Add(const char flag, OptionInfo *optionInfo)
  {
    m_options.insert(OptionMap::value_type(flag, optionInfo));
  }

  OptionInfo *GetOptionInfo(const char flag) const
  {
    OptionMap::const_iterator element = m_options.find(flag);
    if (element != m_options.end())
    {
      return element->second;
    }
    else
    {
      std::cerr << "\nERROR in Options::GetOptionInfo(): "
		<< "-" << flag << " is an invalid option." << std::endl;
      return 0;
    }

  }
private:
  OptionMap m_options;
  std::string m_usageComment;
};


extern int HandleOptions(int argc, char *argv[], 
			 Options &options, int numRequiredArgs,
			 std::string argDescription = "");

extern void Usage(char *argv[], 
		  Options &options, int numRequiredArgs,
		  std::string argDescription = "");

#endif	// _HANDLE_OPTIONS_H_
