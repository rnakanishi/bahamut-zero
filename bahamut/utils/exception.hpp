#ifndef __ARCEUS_UTILS_EXCEPTION_HPP__
#define __ARCEUS_UTILS_EXCEPTION_HPP__

#include <exception>
#include <string>

namespace Arceus {

class ArceusException : virtual public std::exception {
 public:
  ArceusException();
  ArceusException(std::string msg, std::string location);
  ArceusException(int errorId, std::string location);

  ~ArceusException() throw() {}

  /**
   * @brief Get the Error Number for this exception
   *
   * @return int number of the exception
   */
  virtual int getErrorNumber() const throw();

  /**
   * @brief Get the string name for the exception
   *
   * @return const char* points containing the string to the exception name
   */
  virtual const char* getErrorType() const throw();

  /**
   * @brief Returns a pointer to the error description
   *
   * @return const char* the underlying memory that is in possession of the
   * Exception object. Don't attempt to free the memory.
   */
  virtual const char* what() const throw();

  /**
   * @brief Append the provided message to this class message. Useful when
   * recursively calling other methods and tracking becomes complicated
   *
   * @param message
   */
  virtual void appendMessage(std::string message);

  virtual void setErrorNumber(int newErrorNumber, std::string location = "");

  virtual void setLethality(bool lethality);

  virtual void setCallingLocation(std::string location);

  virtual const bool& isLethal();

 protected:
  int _errorNumber;           ///< Error number
  std::string _errorMessage;  ///< Error message
  std::string _callingLocation;
  std::string _exceptionName;
  bool _isLethal = true;
};

class UnexpectedParameterException : public ArceusException {
 public:
  UnexpectedParameterException();
  explicit UnexpectedParameterException(std::string msg, std::string location);
  explicit UnexpectedParameterException(int errorId, std::string location);

  virtual ~UnexpectedParameterException() throw(){};

  virtual void setErrorNumber(int newErrorNumber, std::string location = "");
};

class ConditionsNotMatchException : public ArceusException {
 public:
  ConditionsNotMatchException();

  explicit ConditionsNotMatchException(std::string msg, std::string location);
  explicit ConditionsNotMatchException(int errorId, std::string location);

  virtual ~ConditionsNotMatchException() throw() {}

  virtual void setErrorNumber(int newErrorNumber, std::string location = "");
};

class BadResultException : public ArceusException {
 public:
  BadResultException();

  explicit BadResultException(std::string msg, std::string location);
  explicit BadResultException(int errorId, std::string location);

  virtual ~BadResultException() throw() {}

  virtual void setErrorNumber(int newErrorNumber, std::string location = "");
};

}  // namespace Arceus

#endif