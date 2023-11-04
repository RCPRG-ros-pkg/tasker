import pymongo
import pprint
from bson.json_util import dumps
from pymongo import MongoClient
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *


class dbConnector:
    def __init__(self, db_name):
        self.db_name = "kb_" + str(db_name)
        db_name = self.db_name
        print("Creating DB client")
        self.client = MongoClient()
        dbnames = self.client.list_database_names()
        if db_name in dbnames:
            print "MongoDB DataBase " + str(db_name) + " exists."
        else:
            print "MongoDB DataBase " + str(db_name) + " does not exists."
            self.db = self.client[db_name]
            database_names = self.client.list_database_names()
            print "Successfully created " + str(db_name) + " MongoDB DataBase"
    
    def drop_db(self):
        # db = self.client[self.db_name]
        self.client.drop_database(self.db_name)

    def read_db(self):
        db = self.client[self.db_name]
        functions = db['functions']
        predicates = db['predicates']
        functions_cursor = functions.find({})
        predicates_cursor = predicates.find({})
        functions_docs = []
        predicates_docs = []
        for document in functions_cursor:
            # print(type(document))
            # print(document)
            functions_docs.append(document)
        for document in predicates_cursor:
            predicates_docs.append(document)
        return(predicates_docs, functions_docs)

    def insert_function(self, function):
        functions = self.client[self.db_name].functions
        fun_id = function['_id']
        # if there already is a document of a given id in the collection 
        if functions.count_documents({ '_id': fun_id }, limit = 1) != 0:
            functions.delete_one({'_id': fun_id})
        functions.insert_one(function)
        # self.client.close()

    def insert_predicate(self, predicate):
        predicates = self.client[self.db_name].predicates
        pred_id = predicate['_id']
        # if there already is a document of a given id in the collection 
        if predicates.count_documents({ '_id': pred_id }, limit = 1) != 0:
            predicates.delete_one({'_id': pred_id})
        predicates.insert_one(predicate)
        # self.client.close()

    def print_db(self):
        for doc in self.client.kb.functions.find():
            pprint.pprint(doc)

    def export_db(self, collection):
        cursor = collection.find({})
        with open('collection.json', 'w') as file:
            file.write('[')
            for document in cursor:
                file.write(dumps(document))
                file.write(',')
            file.write(']')

# ki = KnowledgeItem()
# ki.knowledge_type = KnowledgeItem.FUNCTION
# # ki.attribute_name = "battery-level"
# # ki.values.append(diagnostic_msgs.msg.KeyValue("robot", "rico"))
# ki.attribute_name = "distance"
# ki.values.append(diagnostic_msgs.msg.KeyValue("location", "start_point"))
# ki.values.append(diagnostic_msgs.msg.KeyValue("location", "end_point"))
# ki.function_value = 134

# dbc = dbConnector(27)
# dbc.insert_function(ki)