import pickle

def unpickle_dict( filename ):
    """ Return the dictionary stored in the file , Otherwise return an empty dictionary if there were no items """
    try:
        infile = open( filename , 'rb' )
        rtnDict = pickle.load( infile )
        if len( rtnDict ) > 0:
            return rtnDict
        else:
            return {}
    except FileNotFoundError:
        return {}
    
def pickle_struct( pStruct , filename ):
    """ Pickle 'pStruct' to 'filename' """
    outfile = open( filename , 'wb' )
    pickle.dump( pStruct , outfile )
    outfile.close()
    print( "Saved:" , filename )
    
def read_file_as_binary( fPath ):
    """ Open the file in binary mode and return its contents """
    # URL , Pickle and Image: https://mail.python.org/pipermail/python-list/2005-April/313870.html
    f = open( fPath , 'rb' )
    f.seek(0)
    rtnData = f.read()
    f.close()
    return rtnData