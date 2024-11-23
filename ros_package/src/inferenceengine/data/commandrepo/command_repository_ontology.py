from inferenceengine.domain.repos.command_repository import CommandRepository
from inferenceengine.domain.entities import *
from owlready2 import *

class CommandRepositoryOntology(CommandRepository):
    """
    Ontology network implementation of HRIrepositoryAPI
    Code by Marta Fernández Naranjo (mfnar)
    ____________________________________
    """

    __doc__ += CommandRepository.__doc__

    __n_points = None
    __phase = None
    __action = None
    __suture_point = None
    __command = None

    # __action_dict = {
    #     0: Action(grab=True),
    #     1: Action(go_to_stitch=True),
    #     2: Action(stitch=True),
    #     3: Action(stretch_thread=True),
    #     4: Action(grab=True),
    # }

    __action_dict = {
        'agarreAguja': Action(agarreAguja=True),
        'aperturaD': Action(aperturaD=True),
        'aperturaI': Action(aperturaI=True),
        'cierreD': Action(cierreD=True),
        'cierreI': Action(cierreI=True),
        'desplazamiento_cambioD': Action(desplazamiento_cambioD=True),
        'desplazamiento_cambioI': Action(desplazamiento_cambioI=True),
        'desplazamiento_suturaD': Action(desplazamiento_suturaD=True),
        'desplazamiento_suturaI': Action(desplazamiento_suturaI=True),
        'estiramiento_hilo': Action(estiramiento_hilo=True),
        'puncion': Action(puncion=True),
        'None': Action(),
    }


    def initialize(self, initial_phase=None, n_points=0, initial_suture_point=None, java_path='/usr/bin/java', ontology_path='/home/juanmhl/ontology.owl'):

        self.__phase = initial_phase
        self.__n_points = n_points
        self.__suture_point = initial_suture_point
        self.__action = None
        self.__command = Command(self.__phase, self.__action, self.__suture_point, None)

        # Initizliation of the Ontology wrapper
        owlready2.JAVA_EXE = java_path
        self.__ontologia = self.__cargar_ontologia(ontology_path)

        self.__creacion_puntos_sutura(self.__ontologia, self.__n_points)
        self.__establecer_posiblePuntoDeSutura_Siguiente(self.__ontologia, self.__n_points)
    
        nom_punto_final = "ps" + str(self.__n_points)
        self.__punto_final = getattr(self.__ontologia, nom_punto_final)

        self.__sincronizar_razonador(self.__ontologia)

        return True

    def set_phase(self, phase=None):
        """
        This method does not work on the ontology implementation
        """
        return False

    def start(self): 
        """
        This method does not work on the ontology implementation
        """
        return False

    def step(self, predicatesVector):
        # Reasoning step:
        self.__establecer_valor_propiedades_dato(self.__ontologia, predicatesVector, self.__n_points)
        self.__sincronizar_razonador(self.__ontologia)

        # Construction of ontology state
        fase_actual = self.__consulta_fase_actual(self.__ontologia)
        if fase_actual[0:4] == "fase":
            if fase_actual[4] == 'N':
                self.__phase = 0
            else:
                self.__phase = int(fase_actual[4])
        else:
            self.__phase = None
        accion_disponible = self.__consulta_accion_disponible(self.__ontologia)
        punto_sutura_actual = self.__consulta_punto_sutura_actual(self.__ontologia, self.__n_points)
        if punto_sutura_actual[0:2] == "ps":
            self.__suture_point = int(punto_sutura_actual[2])
        else:
            self.__suture_point = None

        acciones_disponibles = accion_disponible.split(" y ")

        if len(acciones_disponibles) == 1:
            self.__action = self.__action_dict[acciones_disponibles[0]]
        elif len(acciones_disponibles) == 2:
            self.__action = self.__action_dict[acciones_disponibles[0]] + self.__action_dict[acciones_disponibles[1]]

        # self.__action = self.__action_dict[accion_disponible]
        self.__command = Command(self.__phase, self.__action, self.__suture_point, None)

        # Ontology actualization
        if self.__phase != 0: self.__actualizar_ontologia(self.__ontologia)
        
        return self.__command

    def get_phase(self):
        return self.__phase

    def get_action(self):
        return self.__action

    def get_suture_point(self):
        return self.__suture_point

    def get_command(self):
        self.command = Command(self.__phase, self.__action, self.__suture_point, None)
        return self.__command
    
    ########### Functions for managing the ontology ############
    def __cargar_ontologia(self, direccion):
        """
        Función que se descarga la ontología del archivo OWL para poder
        trabajar con ella. Su único argumento es un string con la 
        dirección del archivo. 
        Para que funcione, se tiene que añadir al principio de 
        la dirección "file://". 
        """
        ontologia = get_ontology(direccion).load()
        print("Ontology loaded. ")
        return ontologia

    def __guardar_ontologia(self, ontologia, direccion):
        """
        Función que guarda las modificaciones de la ontología realizadas
        desde Python en un archivo OWL. Su primer argumento es la ontología
        con la que se está trabajando; el segundo, la ruta al archivo OWL.

        Nota 1: Al contrario de la función anterior, no tiene que añadirse 
        "file://" a la dirección.
        Nota 2: No es recomendable utilizar esta función tras haber utilizado
        la función de "actualizar_ontología". El archivo OWL no lo soporta.
        """
        ontologia.save(file = direccion)
        print ("Ontology saved. ")

    def __sincronizar_razonador(self, ontologia):
        """
        Función que activa el razonador, sincronizándolo con las modificaciones
        que se hayan hecho en la ontología. 
        Su único argumento es la ontología.
        """
        with ontologia:
            sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)
        return True

    def __crear_individuo(self, ontologia, nom_clase, nom_individuo):
        """
        Función que crea un individuo nuevo en la ontología y lo asigna a una
        clase dada. Su primer argumento es la ontología; el segundo, un string
        con el nombre de la clase; el tercero, el nombre del individuo a crear.
        """
        with ontologia:
            obj_clase = getattr(ontologia, nom_clase)
            individuo = obj_clase(nom_individuo)
        return individuo

    def __incluir_individuo(self, ontologia, nom_clase, nom_individuo):
        """
        Función que incluye un individuo *ya existente* en una clase.
        Su primer argumento es la ontología; el segundo, un string con el
        nombre de la clase; el tercero, el nombre del individuo ya existente.
        """
        with ontologia:
            obj_clase = getattr(ontologia, nom_clase)
            individuo = getattr(ontologia, nom_individuo)
            individuo.is_a.append(obj_clase)
        return True

    def __establecer_propiedad_objeto(self, ontologia, nom_individuo_1, nom_propiedad, nom_individuo_2):
        """
        Función que establece un rol (object property) entre dos individuos ya existentes.
        El primer argumento es una ontología; el segundo, un string con el nombre del individuo
        perteneciente al dominio; el tercero, un string con el nombre de la propiedad; el cuarto,
        un string con el nombre del individuo pertenciente al rango.
        """
        with ontologia:
            ind1 = getattr(ontologia, nom_individuo_1)
            ind2 = getattr(ontologia, nom_individuo_2)
            prop = getattr(ontologia, nom_propiedad)
            prop[ind1].append(ind2)
        return True

    def __establecer_propiedad_dato(self, ontologia, nom_individuo, nom_prop, dato):
        """
        Función que establece una propiedad de dato a un individuo.
        El primer argumento es la ontología; el segundo, un string con el 
        nombre del individuo; el tercero, un string con el nombre de la propiedad;
        el último, un dato de cualquier tipo permitido por Protegé. 
        El único requisito para el tipo de este último es que coincida con el que
        esté definido en la ontología.
        """
        with ontologia:
            individuo = getattr(ontologia, nom_individuo)
            setattr(individuo, nom_prop, dato)
        return True

    def __consulta_fase_actual(self, ontologia):
        """
        Función que encuentra el nombre de la fase actual de la ontología y lo
        devuelve. 
        Su único atributo es la ontología.
        """
        i = 1
        retval = 'faseNone'
        while (i < 10):
            nom_fase = "fase" + str(i)
            fase_obj = getattr(ontologia, nom_fase)
            if fase_obj.faseActual == True:
                f_actual = fase_obj
                retval = f_actual.name
            i += 1
        return retval
            
    def __consulta_fase_siguiente(self, ontologia):
        """
        Función que encuentra el nombre de la fase siguiente de la ontología.
        Su único atributo es la ontología.
        """
        i = 1
        retval = 'faseNone'
        while (i < 10):
            nom_fase = "fase" + str(i)
            fase_obj = getattr(ontologia, nom_fase)
            if fase_obj.faseSiguiente == True:
                fase_siguiente = fase_obj
                retval = fase_siguiente.name
            i += 1
        return retval
    
    def __consulta_punto_sutura_actual(self, ontologia, num_puntos):
        """
        Función que encuentra el punto de sutura actual de la ontología y devuelve
        su nombre.
        Su primer atributo es la ontología; el segundo, el número de puntos de la ontología.
        """
        i = 1
        retval = 'psNone'
        while(i <= num_puntos):
            nom_punto = "ps" + str(i)

            punto = getattr(ontologia, nom_punto)
            
            if (punto.puntoSuturaActual == True):
                punto_actual = punto
                retval = punto_actual.name
            i += 1
        return retval


    def __consulta_accion_disponible(self, ontologia):
        """
        Función que sirve para la consulta de la acción a ejecutar.
        Busca en la ontología la fase actual y devuelve la acción que se corresponda con ella.
        Si la lista tiene solo un elemento, devuelve el nombre directamente. Si tiene más de uno,
        une los dos en un string y posteriormente lo devuelve.
        Su único argumento es la ontología.
        """
        with ontologia:
            fase_actual_nom = self.__consulta_fase_actual(ontologia)

            if fase_actual_nom == "faseNone":
                accion_disponible_nom = "None"
                return accion_disponible_nom
            
            else:
                fase_actual_obj = getattr(ontologia, fase_actual_nom)

                if fase_actual_obj.tieneAccion[0] == fase_actual_obj.tieneAccion[-1]:
                    # (si son iguales, solo hay una acción)
                    accion_disponible_nom = fase_actual_obj.tieneAccion[0].name
                else:
                    accion_disponible_1_nom = fase_actual_obj.tieneAccion[0].name
                    accion_disponible_2_nom = fase_actual_obj.tieneAccion[1].name
                    accion_disponible_nom = accion_disponible_1_nom + " y " + accion_disponible_2_nom
                
                return accion_disponible_nom

    def __actualizar_ontologia(self, ontologia):
        """
        Función que actualiza la ontología, convirtiendo la fase actual en
        fase visitada, y la fase siguiente en una fase normal.
        Su único argumento es la ontología.
        """
        nom_fase_actual = self.__consulta_fase_actual(ontologia)
        nom_fase_siguiente = self.__consulta_fase_siguiente(ontologia)

        self.__establecer_propiedad_dato(ontologia, nom_fase_actual, "faseVisitada", True)
        self.__establecer_propiedad_dato(ontologia, nom_fase_actual, "faseActual", None)
        self.__establecer_propiedad_dato(ontologia, nom_fase_siguiente, "faseSiguiente", None)

        if nom_fase_actual == "fase9":
            i = 1
            while (i < 10):
                nom_fase = "fase" + str(i)
                self.__establecer_propiedad_dato(ontologia, nom_fase, "faseVisitada", False)
                i += 1
        return True      
            
    def __creacion_puntos_sutura(self, ontologia, num_puntos):
        """
        Función que crea el número de puntos de sutura introducidos
        como individuos de la ontología. 
        El primer argumento es la ontología; el segundo, un entero que 
        indica el número de puntos de la operación.
        """
        with ontologia:
            i = 0
            while i < num_puntos:
                i += 1
                nom_punto = "ps" + str(i)
                clase_punto = getattr(ontologia, "PuntoDeSutura")
                ind_punto = clase_punto(nom_punto)
                self.__establecer_propiedad_dato(ontologia, nom_punto, "puntoTerminado", False)
                self.__establecer_propiedad_dato(ontologia, nom_punto, "libre", True)
        return True

    def __establecer_posiblePuntoDeSutura_Siguiente(self, ontologia, num_puntos):
        """
        Función que establece la propiedad de "posiblePuntoDeSutura_Siguiente"
        entre los puntos de sutura. Estos tienen que estar definidos previamente.
        Su primer argumento es la ontología; el segundo, un entero con el número
        de puntos de la intervención. 
        """
        with ontologia:
            i = 1
            while i < num_puntos:
                nom_primer_punto = "ps" + str(i)
                nom_segundo_punto = "ps" + str(i+1)
                self.__establecer_propiedad_objeto(ontologia, nom_primer_punto, "posiblePuntoDeSutura_Siguiente", nom_segundo_punto)
                i += 1
            nom_ultimo_punto = "ps" + str(num_puntos)
            self.__establecer_propiedad_objeto(ontologia, nom_ultimo_punto, "posiblePuntoDeSutura_Siguiente", nom_ultimo_punto)
        return True

    def __establecer_valor_propiedades_dato(self, ontologia, lista, num_puntos):
        """
        Función que establece el valor de todas las propiedades de dato en función de 
        una lista que se introduce. 
        Su primer argumento es la ontología; el segundo, una lista de siete elementos
        con las siguientes propiedades:
        1. un booleano que se corresponda con cerradoPinza derecha.
        2. un booleano que se corresponda con cerradoPinza izquierda.
        3. un booleano que se corresponda con enPosicionCambioD en el punto actual.
        4. un booleano que se corresponda con enPosicionCambioI en el punto actual.
        5. un booleano que se corresponda con enPosicionPinzaD en el punto actual.
        6. un booleano que se corresponda con enPosicionPinzaI en el punto actual.
        7. un booleano que se corresponda con libre en el punto actual.
        8. una lista con el valor booleano de "puntoTerminado" en todos los puntos.

        El tercer argumento, el número de puntos de la ontología

        NOTA: la lista de predicados aparece aquí de la forma más sencilla para poder 
        hacer la prueba del programa entero. Luego tendré que adaptarlo a lo que hayan
        desarrollado Juanma y Álvaro, que lo he intentado mirar en Github y no me entero de nada.
        """

        nom_punto_actual = self.__consulta_punto_sutura_actual(ontologia, num_puntos)

        self.__establecer_propiedad_dato(ontologia, "d", "cerradoPinza", lista[0])
        self.__establecer_propiedad_dato(ontologia, "i", "cerradoPinza", lista[1])
        self.__establecer_propiedad_dato(ontologia, nom_punto_actual, "enPosicionCambioD", lista[2])
        self.__establecer_propiedad_dato(ontologia, nom_punto_actual, "enPosicionCambioI", lista[3])
        self.__establecer_propiedad_dato(ontologia, nom_punto_actual, "enPosicionPinzaD", lista[4])
        self.__establecer_propiedad_dato(ontologia, nom_punto_actual, "enPosicionPinzaI", lista[5])
        self.__establecer_propiedad_dato(ontologia, nom_punto_actual, "libre", lista[6])
        
        num = 1
        for i in lista[7]:
            nom_punto = "ps" + str(num)
            self.__establecer_propiedad_dato(ontologia, nom_punto, "puntoTerminado", i)
            num += 1
        return True

    def __introducir_lista_predicados(self, num_puntos):
        """
        Función que pide la lista de predicados al usuario.
        Su único argumento es el número de puntos de la ontología.
        
        Esta función es provisional para la prueba del código,
        porque luego tendré que adaptarlo a las clases estas
        de Álvaro y Juanma
        """
        n = 7
        lista = []
        lista_puntoTerminado = []
        
        for i in range(0, 7):
            respuesta = input()
            if respuesta == "True" or respuesta == "1":
                valor = True
            if respuesta == "False" or respuesta == "0":
                valor = False

            lista.append(valor)
        
        for i in range(0, num_puntos):
            respuesta = input()
            if respuesta == "True" or respuesta == "1":
                valor = True
            if respuesta == "False" or respuesta == "0":
                valor = False

            lista_puntoTerminado.append(valor)
        
        lista.append(lista_puntoTerminado)

        return lista 


if __name__ == "__main__":
    initial_phase = Phase(0)
    n_points = 3
    initial_suture_point = 0
    repo = CommandRepositoryOntology()
    repo.initialize(initial_phase,n_points,initial_suture_point)
    print(repo.get_command())

    # First step
    repo.step([True, False, False, False, False, False, True, [False, False, False]])
    print(repo.get_command())