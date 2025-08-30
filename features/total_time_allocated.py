#This module tracks the total time spent on a particular skill.
#Skills_tracker

#skill_name = name of the skill
#time_spent = give the time that we allocated on a particular a skill.
#session = A session is simply time spent from start to end on a specific date.

'''
___________________________________________
Schema for the table "Skills":             |
CREATE TABLE IF NOT EXISTS Skills(         |
    SkillID INTEGER PRIMARY KEY,           |
    Name TEXT NOT NULL,                    |
    CreatedOn TEXT,                        |
    TotalTimeSpent INTEGER                 |
    );                          |          |
___________________________________________|
Schema for the table "Sessions":           |
CREATE TABLE IF NOT EXISTS Sessions(       |
    SessionID INTEGER PRIMARY KEY,         |
    SkillID INTEGER,                       |
    StartTime TEXT,                        |
    EndTime TEXT,                          |
    Duration INTEGER,                      |
    Notes TEXT,                            |
    FOREIGN KEY (SkillID) REFERENCES Skills|
    (SkillID) ON DELETE CASCADE            |
    );                                     |
___________________________________________|
'''
#imports
from .. import custom_exception as ce
import sqlite3
from datetime import datetime
conn = sqlite3.connect("../database/mind.db") #connect to the database
cur = conn.cursor() #create a cursor object
cur.execute("PRAGMA foreign_keys = ON;") #enable foreign key support

#TODO we might call these functions in the main.py file so importing and connection should happen there.


def create_skill(skill_name:str) -> bool:
    """Take the name of the skill from the user and inserts in the database."""
    try:
        cur.execute("SELECT 1 FROM Skills WHERE Name = ?", (skill_name,))
        exists = cur.fetchone()
        if exists:
            raise ce.SkillAlreadyExistsError(f"skill with name {skill_name} already exists.")
            
        # insert into the table
        cur.execute("""
        INSERT INTO Skills (Name, CreatedOn, TotalTimeSpent) VALUES (?, DATE('now'), 0)""", (skill_name,))
        return True
    except ce.SkillAlreadyExistsError as sae:
        print(sae)
        print("Please choose a different skill name.")
        return False
    except sqlite3.Error as e:
        print(f"Database error: {e}")
        return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False



def get_all() -> str:
    """This function returns all the skills and time allocated."""
    pass

def get(skill_name:str) -> str:
    """This function takes the skill name as input and returns the details.
    if session is true then session is also included in the output."""
    pass

#TODO we can implement a function that takes constraint and returns the data from the table. see below
"""query across multiple skills or dates. Example: Show me total time spent across all skills between 2025-08-01 and 2025-08-10."""

def start_session(skill_name:str) -> bool:
    """ This function starts a session and return true if it is successful"""
    # try:
    #     cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
    #     skill_id = cur.fetchone()
    #     if not skill_id:
    #         raise ce.SkillNotFoundError(f"Skill with name {skill_name} does not exist.")
        
    #     skill_id = skill_id[0]
    #     start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
def end_session(skill_name:str) -> bool:
    """This function ends a session and saves it in the table and return true if is succesful"""
    pass

def pause_session(tracker:str|int) -> bool:
    """ If we are taking break then we use this function"""
    pass

def delete_skill(skill_name:str) -> bool:
    """This function with taking the skill name as input, deletes the skill and its data from the table."""
    try:
        cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
        skill_id = cur.fetchone()
        if not skill_id:
            raise ce.SkillNotFoundError(f"Skill with name {skill_name} does not exist.")
        cur.execute("DELETE FROM Skills WHERE SkillID = ?", (skill_id[0],))

        
    except ce.SkillNotFoundError as snf:
        print(snf)
        print("Please choose correct skill name.")
        return False
    except sqlite3.Error as e:
        print(f"Database error: {e}")
        return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False

def reset_skill(skill_name:str, not_date:bool=True) -> bool:
    """This function keeps the skill and deletes all the data about the skill. If not_date is true then it keeps the created date else it resets the created date also."""
    try:
        cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
        skill_id = cur.fetchone()
        if not skill_id:
            raise ce.SkillNotFoundError(f"Skill with name {skill_name} does not exist.")
        if not_date:
            cur.execute("UPDATE Skills SET CreatedOn = DATE('now'), TotalTimeSpent = 0 WHERE SkillID = ?", (skill_id[0],))
        else:
            cur.execute("UPDATE Skills SET TotalTimeSpent = 0 WHERE SkillID = ?", (skill_id[0],))

    except ce.SkillNotFoundError as snf:
        print(snf)
        print("Please choose correct skill name.")
        return False
    except sqlite3.Error as e:
        print(f"Database error: {e}")
        return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False