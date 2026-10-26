"""RoboClean AI: simulador educativo de un robot aspirador autónomo.
Python 3.10+; únicamente biblioteca estándar (Tkinter).
"""
import tkinter as tk
from tkinter import ttk
import math
import random
import heapq
from collections import deque

CELL = 28
W, H = 22, 16
INITIAL_OBSTACLES = {(x, y) for x in range(W) for y in range(H) if x in (0,W-1) or y in (0,H-1)}
INITIAL_OBSTACLES |= {(6,y) for y in range(3,8)} | {(x,5) for x in range(12,17)}
INITIAL_OBSTACLES |= {(15,y) for y in range(10,14)} | {(x,11) for x in range(4,8)}
START = (2,2)
DUST = {(4,4),(9,3),(18,3),(10,8),(18,9),(5,13),(12,13),(19,13)}
MOVE_DIRECTIONS = [(1,0),(-1,0),(0,1),(0,-1)]

class RoboClean:
    def __init__(self, root):
        self.root = root
        self.root.title('RoboClean AI | Simulación de agente inteligente')
        self.root.resizable(False, False)
        self.running = False
        self.speed = tk.IntVar(value=130)
        self.mode = tk.StringVar(value='Automático')
        self.status_var = tk.StringVar()
        self.log_messages = deque(maxlen=7)
        self.build_ui()
        self.reset()
        self.root.after(200, self.loop)

    def build_ui(self):
        container = ttk.Frame(self.root, padding=12)
        container.pack()
        self.canvas = tk.Canvas(container, width=W*CELL, height=H*CELL, bg='#f4f7fc', highlightthickness=1, highlightbackground='#aab5c4')
        self.canvas.grid(row=0, column=0, rowspan=2)
        side = ttk.Frame(container, padding=(15,0,0,0))
        side.grid(row=0, column=1, sticky='n')
        ttk.Label(side,text='ROBOCLEAN AI',font=('Arial',16,'bold')).pack(anchor='w')
        ttk.Label(side,text='Agente autónomo • Mapeo probabilístico',foreground='#536273').pack(anchor='w',pady=(0,10))
        self.metrics = ttk.Label(side, text='', font=('Consolas',10), justify='left')
        self.metrics.pack(anchor='w')
        ttk.Separator(side).pack(fill='x',pady=8)
        ttk.Label(side,text='CONTROLES',font=('Arial',10,'bold')).pack(anchor='w')
        controls = ttk.Frame(side)
        controls.pack(anchor='w',pady=6)
        ttk.Button(controls,text='▶ Iniciar',command=self.start).grid(row=0,column=0,padx=2,pady=2)
        ttk.Button(controls,text='⏸ Pausar',command=self.pause).grid(row=0,column=1,padx=2,pady=2)
        ttk.Button(controls,text='↺ Reiniciar',command=self.reset).grid(row=1,column=0,padx=2,pady=2)
        ttk.Button(controls,text='⚠ Anomalía',command=self.add_obstacle).grid(row=1,column=1,padx=2,pady=2)
        ttk.Label(side,text='Intervalo de simulación (ms)').pack(anchor='w')
        ttk.Scale(side,from_=70,to=450,variable=self.speed,orient='horizontal').pack(fill='x',pady=(0,9))
        ttk.Label(side,text='VISTA DEL MAPA',font=('Arial',10,'bold')).pack(anchor='w')
        for s in ['Gris oscuro: obstáculo real', 'Azul: probabilidad de ocupación', 'Amarillo: suciedad', 'Verde: recorrido', 'Rojo: anomalía detectada']:
            ttk.Label(side,text='• '+s).pack(anchor='w')
        ttk.Separator(side).pack(fill='x',pady=10)
        ttk.Label(side,text='EVENTOS DEL AGENTE',font=('Arial',10,'bold')).pack(anchor='w')
        self.events = tk.Text(side, width=39,height=9,wrap='word',state='disabled',font=('Consolas',9))
        self.events.pack(pady=(4,0))
        ttk.Label(container,text='Clic en habitación: añadir o retirar obstáculo inesperado | Sensores: 4 rayos, alcance 5 celdas | A*: planificación',foreground='#526170').grid(row=2,column=0,columnspan=2,sticky='w',pady=(9,0))
        self.canvas.bind('<Button-1>',self.toggle_obstacle)

    def reset(self):
        self.running = False
        self.obstacles = set(INITIAL_OBSTACLES)
        self.known_dust = set(DUST)
        self.dust = set(DUST)
        self.position = START
        self.history = [START]
        self.steps = 0
        self.battery = 100.0
        self.anomalies = 0
        self.last_anomaly = None
        self.cleaned = 0
        self.belief = [[0.5 for _ in range(W)] for _ in range(H)]
        self.target = None
        self.route = []
        self.log_messages.clear()
        self.log('Inicio: mapa desconocido (P=50%).')
        self.sense()
        self.plan()
        self.draw()

    def log(self, message):
        self.log_messages.appendleft(message)
        self.events.configure(state='normal')
        self.events.delete('1.0','end')
        self.events.insert('end','\n'.join(self.log_messages))
        self.events.configure(state='disabled')

    def start(self):
        self.running = True
        self.log('Exploración y limpieza iniciadas.')

    def pause(self):
        self.running = False
        self.log('Simulación pausada.')
        self.draw()

    def in_bounds(self, pos):
        return 0 <= pos[0] < W and 0 <= pos[1] < H

    def update_belief(self, cell, measurement_occupied):
        """Actualización bayesiana en log-odds con sensores binarios simulados.
        Sensibilidad/especificidad nominal de 0.90; mediciones generadas con ruido.
        """
        x,y = cell
        prior = min(0.995,max(0.005,self.belief[y][x]))
        log_odds = math.log(prior/(1-prior))
        evidence = math.log(0.9/0.1) * (1 if measurement_occupied else -1)
        updated = max(-5,min(5,log_odds+evidence))
        self.belief[y][x] = 1/(1+math.exp(-updated))

    def sense(self):
        """Cuatro sensores de alcance: trazan rayos e incorporan un 10% de error binario.
        Es un modelo didáctico simplificado, no LIDAR físico ni SLAM completo.
        """
        x,y = self.position
        self.update_belief((x,y),False)
        for dx,dy in MOVE_DIRECTIONS:
            for k in range(1,6):
                pt = (x+dx*k,y+dy*k)
                if not self.in_bounds(pt):
                    break
                actual = pt in self.obstacles
                reading = actual if random.random() < 0.90 else not actual
                self.update_belief(pt,reading)
                if actual:  # el rayo real se bloquea físicamente; modelo simplificado
                    break

    def astar(self,start,goal):
        pq = [(0,start)]
        cost = {start:0}
        came = {}
        while pq:
            _,u = heapq.heappop(pq)
            if u == goal:
                path=[]
                while u != start:
                    path.append(u)
                    u=came[u]
                return list(reversed(path))
            for dx,dy in MOVE_DIRECTIONS:
                v=(u[0]+dx,u[1]+dy)
                if not self.in_bounds(v) or self.belief[v[1]][v[0]] >= 0.70:
                    continue
                # Mayor costo al atravesar lugares que todavía son desconocidos.
                uncertainty = self.belief[v[1]][v[0]]
                alt=cost[u] + 1 + 1.5*uncertainty
                if alt < cost.get(v,float('inf')):
                    cost[v]=alt
                    came[v]=u
                    h=abs(goal[0]-v[0])+abs(goal[1]-v[1])
                    heapq.heappush(pq,(alt+h,v))
        return None

    def plan(self):
        if not self.dust:
            self.target=None
            self.route=[]
            return
        candidates = []
        for d in self.dust:
            route=self.astar(self.position,d)
            if route is not None:
                candidates.append((len(route),d,route))
        if candidates:
            _,target,route=min(candidates)
            if target != self.target:
                self.log(f'Objetivo elegido: {target}. Ruta A*.')
            self.target=target
            self.route=route
        else:
            self.target=None
            self.route=[]
            self.log('No hay ruta conocida; revisar mapa.')

    def tick(self):
        if self.battery <= 0:
            self.running=False
            self.log('ANOMALÍA: batería agotada.')
            return
        self.sense()
        if self.position in self.dust:
            self.dust.remove(self.position)
            self.cleaned+=1
            self.log(f'Área limpiada: {self.position}.')
        if not self.dust:
            self.running=False
            self.log('OBJETIVO CUMPLIDO: limpieza terminada.')
            return
        self.plan()
        if not self.route:
            self.running=False
            return
        candidate=self.route[0]
        # Comprobación física de colisión: ante objeto no anticipado se corrige creencia.
        if candidate in self.obstacles:
            if self.last_anomaly != candidate:
                self.anomalies+=1
                self.last_anomaly=candidate
                self.log(f'ANOMALÍA: obstáculo en {candidate}.')
            for _ in range(3):
                self.update_belief(candidate,True)
            self.plan()
            return
        self.position=candidate
        self.history.append(candidate)
        self.steps+=1
        self.battery=max(0,self.battery-0.28)
        if self.battery <= 20 and self.battery+0.28 > 20:
            self.anomalies+=1
            self.log('ANOMALÍA: batería baja (< 20%).')
        self.sense()

    def add_obstacle(self):
        candidates=[]
        px,py=self.position
        for dx,dy in MOVE_DIRECTIONS:
            p=(px+dx*2,py+dy*2)
            if 0<p[0]<W-1 and 0<p[1]<H-1 and p not in self.obstacles and p not in self.dust:
                candidates.append(p)
        if self.route:
            candidates=[p for p in self.route[1:5] if p not in self.dust and p not in self.obstacles] + candidates
        if candidates:
            p=candidates[0]
            self.obstacles.add(p)
            self.log(f'Obstáculo inesperado colocado: {p}.')
            self.draw()

    def toggle_obstacle(self,event):
        p=(event.x//CELL,event.y//CELL)
        if not (0<p[0]<W-1 and 0<p[1]<H-1) or p==self.position or p in self.dust:
            return
        if p in self.obstacles:
            self.obstacles.remove(p)
            self.log(f'Obstáculo retirado: {p}.')
        else:
            self.obstacles.add(p)
            self.log(f'Obstáculo nuevo: {p}.')
        self.draw()

    def loop(self):
        if self.running:
            self.tick()
            self.draw()
        self.root.after(max(70,int(self.speed.get())), self.loop)

    def draw(self):
        c=self.canvas
        c.delete('all')
        for y in range(H):
            for x in range(W):
                p=self.belief[y][x]
                # Blanco: libre conocido; azul: incertidumbre; negro: ocupado inferido.
                if p >= .7:
                    fill='#566478'
                elif p > .30:
                    fill='#cee0f7'
                else:
                    fill='#f8fbff'
                c.create_rectangle(x*CELL,y*CELL,(x+1)*CELL,(y+1)*CELL,fill=fill,outline='#dbe3ed')
                if (x,y) in self.obstacles:
                    c.create_rectangle(x*CELL+4,y*CELL+4,(x+1)*CELL-4,(y+1)*CELL-4,fill='#788394',outline='')
        for x,y in self.history:
            c.create_oval(x*CELL+10,y*CELL+10,x*CELL+18,y*CELL+18,fill='#25a66b',outline='')
        for x,y in self.route:
            c.create_oval(x*CELL+12,y*CELL+12,x*CELL+16,y*CELL+16,fill='#2679d8',outline='')
        for x,y in self.dust:
            c.create_oval(x*CELL+7,y*CELL+7,x*CELL+21,y*CELL+21,fill='#f5b841',outline='#cb8520')
        if self.last_anomaly:
            x,y=self.last_anomaly
            c.create_rectangle(x*CELL+2,y*CELL+2,(x+1)*CELL-2,(y+1)*CELL-2,outline='#ee3c3c',width=3)
        px,py=self.position
        c.create_oval(px*CELL+3,py*CELL+3,(px+1)*CELL-3,(py+1)*CELL-3,fill='#2563eb',outline='#ffffff',width=2)
        c.create_text(px*CELL+CELL/2,py*CELL+CELL/2,text='R',font=('Arial',11,'bold'),fill='white')
        explored=sum(self.belief[y][x]<.3 or self.belief[y][x]>.7 for x in range(W) for y in range(H))
        self.metrics.configure(text=(f'Posición (X,Y):  {self.position}\n'
            f'Batería:          {self.battery:5.1f}%\n'
            f'Pasos:            {self.steps}\n'
            f'Objetivo:         {self.target}\n'
            f'Suciedad retirada:{self.cleaned:3}/{len(DUST)}\n'
            f'Mapa confirmado: {explored/(W*H):5.1%}\n'
            f'Anomalías:        {self.anomalies}\n'
            f'Estado:           {"EN MARCHA" if self.running else "DETENIDO"}'))

if __name__ == '__main__':
    app=tk.Tk()
    RoboClean(app)
    app.mainloop()
