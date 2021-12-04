#!/usr/bin/env python3

#from kubernetes import client, config
#from kubernetes import config, dynamic, client
#from kubernetes.client import api_client
import kubernetes
import pprint
import logging
pp = pprint.PrettyPrinter(indent=4)

def getservices():
    try:
        kubernetes.config.load_incluster_config()
    except:
        print("incluster failed")
        kubernetes.config.load_kube_config()

    """ 
    dynamicclient = kubernetes.dynamic.DynamicClient(
        kubernetes.client.api_client.ApiClient(configuration=kubernetes.config.load_incluster_config())
    )
    dynamicapi = dynamicclient.resources.get(api_version="v1", kind="Service")

    name = "gzserver"
    service = dynamicapi.get(name=name, namespace="simul")
    #pp.pprint(service)
    """
##########
    #values = defaultdict()
    services = []
    v1 = kubernetes.client.CoreV1Api()
    ret = v1.list_namespaced_service(namespace='simul')
    output = "<h4> linkjes </h4> <ul>"
    try:
        for i in ret.items:
            #pp.pprint(i)
            output += '\n <li>'
            output += ' <strong>' + i.metadata.name + '</strong> \n'
            ingress = None
            try: 
                ingress = i.status.load_balancer.ingress[0].ip
            except:
                print('no ingress ip')
            #pp.pprint(i.spec.ports[0])
            
            for p in i.spec.ports:
                output += '<a target="iframe" href="http://' + str(ingress) + ':' + str(p.port) + '"> ' + str(p.name) + ' (' + str(p.port) + ') ' + '</a> \n'
            output += '</li> \n\n'

        output += "</ul>\n"
        print(output) 
    except Exception as e:
        logging.error(e)
        output = '<p> failed: ' + str(e) + '</p>'
        return output
    finally:
        #pp.pprint(jsonify({'values': values,'services':services}))
        return output

def main():
    print(getservices())

if __name__ == '__main__':
    main()