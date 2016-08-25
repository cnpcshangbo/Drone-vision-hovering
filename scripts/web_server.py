import cherrypy
import json
import os
import cv2
import tempfile
from cherrypy.lib.static import serve_file



class Config(object):

    exposed = True

    def __init__(self, config_parser):
        self.config = config_parser

    def get_config(self):
        """ Return a config as a dictionary"""
        dict = {}
        for section in self.config.sections():
            for option in self.config.options(section):
                dict[section + '.' + option] = self.config.get(section, option)

        return dict

    def handle_set(self, id, newval):
        split = id.split('.')
        section = split[0]
        option = split[1]
        self.config.set(section, option, newval)
        return "%s has been set to %s" % (id, newval)

    def GET(self, id=None, set=None):
        if id == None:
            return json.dumps(self.get_config())
        else:
            # We allow the user to do 'sets' via a query param of the form ?set=value
            #setparam = cherrypy.request.params.get('set')
            if set != None:
                return self.handle_set(id, set)
            else:
                return self.get_config()[id]    

class Image(object):

    exposed = True

    def __init__(self, image_callback):
        self.image_file = tempfile.mktemp(suffix=".jpg")
        self.image_callback = image_callback

    def GET(self, id=None):
        """Write our image to a temp file, then give it back to the user"""
        cv2.imwrite(self.image_file, self.image_callback())
        return serve_file(self.image_file, content_type='image/jpeg')
        
class Image_masked(object):

    exposed = True

    def __init__(self, image_callback):
        self.image_file = tempfile.mktemp(suffix=".jpg")
        self.image_callback = image_callback

    def GET(self, id=None):
        """Write our image to a temp file, then give it back to the user"""
        cv2.imwrite(self.image_file, self.image_callback())
        return serve_file(self.image_file, content_type='image/jpeg')

class Static:
    def __init__(self):
        self.max=14
        self.min=3
        self.smax=255
        self.smin=20
        self.vmax=255
        self.vmin=20
    def index(self):
        html = """
        <html>
            <head>
                <META HTTP-EQUIV="refresh" CONTENT="1">
            </head>
            <body>
                <h1>AVC Red balloon ass kicker</h1>
                <!-- <form action="/process_form/" method="post">
                    <h2>Config</h2>
                    config1.foo <input type="text" name="login" value="login" /><br>
                    config1.foo <input type="text" name="login" value="login" />
                    <input type="submit" />
                </form> 
                <canvas id="my_canvas" width="700" height="480"></canvas>-->
                <font size="5" color="red">
                <a href="tune">Start Tuning</a>
                </br>
                <img src="/image" />

                </font>

            </body>
        </html>
        """
        return html

    index.exposed = True
    
    @cherrypy.expose
    def tune(self):
        return open('show_threshold.html')
        
    @cherrypy.expose
    def update_threshold(self,max,min,smax,smin,vmax,vmin):
        self.max=max
        self.min=min
        self.smax=smax
        self.smin=smin
        self.vmax=vmax
        self.vmin=vmin
        
        return '''<h1>Parameters updated. Return to <a href=index>home</a><h1>'''
    
    

class Webserver(object):
    def __init__(self, config_parser, image_callback, image_masked):
        cherrypy.tree.mount(
            Config(config_parser), '/config',
            {'/': {'request.dispatch': cherrypy.dispatch.MethodDispatcher()} } )
        cherrypy.tree.mount(
            Image(image_callback), '/image',
            {'/': {'request.dispatch': cherrypy.dispatch.MethodDispatcher()} } )
        cherrypy.tree.mount(
            Image_masked(image_masked), '/image_masked',
            {'/': {'request.dispatch': cherrypy.dispatch.MethodDispatcher()} } )
        my_static=Static()
        cherrypy.tree.mount(my_static, '/')

        cherrypy.config.update({
                         'server.socket_port': 8081,
                         'server.socket_host': '0.0.0.0',
                         'log.screen': None
                        }) 

        cherrypy.engine.start()
        # cherrypy.engine.block()
        self.my_static = my_static
        print 'max in webserver: ', self.my_static.max

        print "Cherrypy initial completed."
        
    def close(self):
        cherrypy.engine.stop()

if __name__ == '__main__':

    Webserver()
    cherrypy.engine.block()