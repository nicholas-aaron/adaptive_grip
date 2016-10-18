/*
	Name			: 	basic_shell.cpp
	Author		: 	Aaron
	Purpose		:	Implement a "shell" program that can be used to
						send the Gantry robot some basic commands
*/

#include <string>
#include <list>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <sstream>

extern "C" {
#include <stdio.h>
#include "readline/readline.h"
#include "readline/history.h"
}

#include "Robot.h"

using namespace std;

typedef void 				handler ( list<string>& args, Robot& robot );
typedef list<string> 	arg_list;


// This was an ofstream being used to log everything.
ofstream						Log("shell.log");

string						Prompt = "$ ";
string						cmdline;

bool quit = false; // Set to true when we're done

typedef struct {
	string 							name; // Function Name, to users
	handler	* 						func; // Handler Function Pointer
	string							doc; 	// Documentation
} Command;


typedef vector<Command>				command_list;
typedef command_list::iterator 	command_iterator;
list<string> split (const string& s);

command_list	commands;
handler			hl_help, hl_quit; // Basic
handler 			hl_move_xyz, hl_move_abc; // Move 'XYZ', Move 'ABC'


list<string> split (const string& s) {
	istringstream line(s);
	string word;
	list<string> words;
	while (line >> word) words.push_back(word);
	return words;
}

/*
	hl_help: "Help" handler
*/
void hl_help(arg_list & args, Robot & robot)
{
	cout << "This is a help function!" << endl;
}

void hl_quit(arg_list & args, Robot & robot)
{
	cout << "You are a quitter!" << endl;
	quit = true;
}

void hl_move_xyz(arg_list & args, Robot & robot)
{
///cout << "hl_move_xyz: This doesn't do anything yet." << endl;
	cout << "hl_move_xyz: Num args = " << args.size() << endl;
	if (args.size() != 3) {
		cout << "hl_move_xyz: Need 3 arguments. Exiting..." << endl;
	}

	else {
		double xpos, ypos, zpos;
		xpos = atof(args.front().c_str());
		args.pop_front();
		ypos = atof(args.front().c_str());
		args.pop_front();
		zpos = atof(args.front().c_str());
		args.pop_front();

		RobotPosition current_pos = robot.currentPos();
		RobotPosition new_pos = RobotPosition(xpos, ypos, zpos, 0, 0, 0);
		new_pos.j4 = current_pos.j4;
		new_pos.j5 = current_pos.j5;
		new_pos.j6 = -90.0;



		cout << "hl_move_xyz - Current Position: " << current_pos << endl;
		cout << "hl_move_xyz - Next    Position: " << new_pos << endl;

		robot.moveTo(new_pos);
	} 
}

void hl_move_abc(arg_list & args, Robot & robot)
{
	cout << "hl_move_abc: This doesn't do anything yet." << endl;
}

void hl_get_pos(arg_list & args, Robot & robot)
{
	cout << "Robot Position: " << robot.currentPos() << endl;
}

void hl_get_limits(arg_list & args, Robot & robot)
{
	cout << robot.currentLimits() << endl;
}


void construct_commands() 
{
	// Initialize it
	commands = command_list();

	commands.push_back({ "help", 	hl_help, "Display help." });
	commands.push_back({ "?",		hl_help, "Display help." });
	commands.push_back({ "grid",	hl_move_xyz,	"Move to an x-y-z position." });
	commands.push_back({ "quit",	hl_quit,	"Quit Program." });
	commands.push_back({ "pos",	hl_get_pos,	"Get Position." });
	commands.push_back({ "lim",	hl_get_limits,	"Get Limits." });

}

void execute_command(const string& command, Robot & robot)
{
	// Split the command into a list of arguments and a command name
	arg_list		args 			= split(command);
	string 		name     	= args.front();
	args.pop_front();

	bool			found 		= false;

	int i = 0;

	// cout << "execute_command(): Name = <" << name << ">" << endl;

	// Search for a matching command
	command_iterator cmd;
	for (cmd = commands.begin(); cmd != commands.end(); cmd++)
	{
		if ((*cmd).name == name) {
			((*cmd).func)(args, robot);
			found = true;
		}
	}

	if (!found) {
		cout << "Command <" << name << "> not found." << endl;
	}
	



}

string & trim (string & s)
{
	size_t pos = s.find_first_not_of(" \t");
	if (string::npos != pos)
	{
		s = s.substr(pos);
	}
	pos = s.find_last_not_of(" \t");
	if (string::npos != pos)
	{
		s = s.substr(0, pos+1);
	}
	return s;
}

int main(int argc, char * argv[])
{
	// TODO assign rl_completion_entry_function
	// and get command autocomplete working

	construct_commands();

	// Initialize the robot
	Robot robot("/dev/gantry");
	robot.home();

	while (!quit) {
		char * input = readline(Prompt.c_str());

		if (input) {

			cmdline = string(input);
			trim(cmdline);
			execute_command(cmdline, robot);

			// Free it, null it
			free(input);
			input = (char *) NULL;

		}

		else {
			cmdline = "exit";
			cout << "Exiting! (DEBUG: (!input) == TRUE)" << endl;
		}

		if (cmdline.size() <= 0) continue; // Ignore blank lines

	}

}

// Trim

