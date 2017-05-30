
#pragma once

#include "./3rd_Party/tcpsockets/tcpacceptor.h"

// From: https://github.com/nlohmann/json
#include <./3rd_Party/json.hpp>
using json = nlohmann::json;

// See Robot.cpp for these functions:
extern void addRobotCommand(json cmd);
extern json getDriveMacros();
extern void putDriveMacros(json &macros_j);

namespace GM_Code
{

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// gmServer
//........................................
class gmServer
{
  public:
    static void StartListening()
    {
    	const int msg_getDrive = 1001, msg_getMacros = 1002, msg_putMacros = 1003, msg_addCmd = 1004;

        TCPAcceptor *acceptor = new TCPAcceptor(5800);

        if (acceptor->start() == 0)
        {
            while (true)
            {
            	TCPStream *stream = acceptor->accept();

                if (stream != NULL)
                {
                	char str[80];
                	sprintf(str, "Data Received");
                	DriverStation::ReportError(str);

                    ssize_t len;
                    std::vector<char> buffer;
                    char line[256];
                    int size = 0;

                    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                    // Receive the entire message.
                    //............................................................
                    while ((len = stream->receive(line, sizeof(line), 5/*5s timeout*/)) > 0)
                    {
                    	for (auto i = 0; i < len; i++)
                    		buffer.push_back(line[i]);
                    	size += len;
                    }

                    //std::string message(buffer.begin(), buffer.end());
					json msg_r = json::parse(buffer);

					switch ((int)msg_r["id"]) //Look in the start of this function for the ID #'s
					{
						case msg_addCmd:
							{
								char str[80];
								sprintf(str,"Command Received : Size = %d", size);
								DriverStation::ReportError(str);

								addRobotCommand(msg_r);
							}
							break;

						case msg_getDrive:
							break;

						case msg_getMacros:
							{
								std::string msg_str = msg_r.dump();
								DriverStation::ReportError(msg_str);

								std::string s = getDriveMacros().dump();
								stream->send(s.c_str(), s.length());

			                    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
								// Send a copy of the received msg to say we're done.
			                    //............................................................
								stream->send(msg_str.c_str(), msg_str.length());
							}
							break;

						case msg_putMacros:
							{
								char str[80];
								sprintf(str, "putMacros size = %d", size);
								DriverStation::ReportError(str);

								putDriveMacros(msg_r["macros"]);
			                    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
								// Not providing a response, here; is that wise? (?)
			                    //............................................................
							}
							break;
					}

                    delete stream;
                }
            }
        }
    }
};

} // namespace GM_Code
