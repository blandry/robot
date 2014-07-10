/*!
 * \file CmdBase.h
 *
 * \author Rodrigues Filho
 * \version 0.1
 *
 * \section LICENSE
 * TODO
 * \todo Make the license
 *
 * \section DESCRIPTION
 * This provides a base interface for commands in this CmdMessenger C++ implementation.
 * Here, the empty class CmdEnd is also defined.
 */

#ifndef CMD_BASE_H
#define CMD_BASE_H

/*!
 * An empyt class that indicates the end of a command
 */
class CmdEnd{};

/*!
 * Base class for commands (Send and Received commands).
 */
class CmdBase
{
  public:
    CmdBase(int id = 0);
    virtual ~CmdBase();

    void setId(int id);
    int getId() const;

  private:
    int id_;
};

#endif
