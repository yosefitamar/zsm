import psutil
import time
import subprocess
import os
import re

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.live import Live
from rich.layout import Layout
from rich.text import Text
from rich.align import Align
import requests

# --- Variáveis Globais para Otimização ---
NUM_CPUS_LOGICAL = psutil.cpu_count(logical=True)
NUM_CPUS_PHYSICAL = psutil.cpu_count(logical=False)

# Listas de exclusão (inicialmente vazias para mostrar tudo)
# Adicione nomes de usuários que você NÃO deseja ver na lista de processos.
EXCLUDE_USERS = [] 

# Adicione substrings de nomes de processos que você NÃO deseja ver.
# Ex: ['systemd', 'kworker', 'gnome-shell']
EXCLUDE_PROCESS_NAMES = [
    # --- Processos de Kernel / systemd ---
    'systemd',              # Sistema de init principal
    'kworker',              # Threads de trabalho do kernel
    'kdevtmpfs',            # Kernel device temporary filesystem
    'ksoftirqd',            # Daemon de softirqs do kernel
    'kthreadd',             # Daemon de threads do kernel
    'rcu_gp',               # Threads de RCU (Read-Copy Update)
    'rcu_tasks_',           # Para excluir processos como 'rcu_tasks_kthread'
    'migration',            # Threads de migração de CPU
    'cpuhp',                # CPU hotplug daemon
    'kauditd',              # Daemon de auditoria do kernel
    'systemd-journal',      # Serviço de logging do systemd
    'systemd-udevd',        # Gerenciador de dispositivos (udev)
    'systemd-logind',       # Gerencia sessões de login
    'systemd-resolved',     # Resolvedor de DNS
    'systemd-networkd',     # Configuração de rede (alternativa ao NetworkManager)
    'systemd-timesyncd',    # Sincronização de tempo (NTP)
    'dbus-daemon',          # Sistema de comunicação interprocessos
    'irqbalance',           # Otimiza distribuição de IRQs

    # --- Serviços Básicos / Daemons (exemplo) ---
    'cron',                 # Agendador de tarefas
    'sshd',                 # Servidor OpenSSH (daemon)
    'snapd',                # Daemon do gerenciador de pacotes Snap
    'bash',                 # Shell, se não for sua sessão interativa principal
    'rcu_',                 # Exclui todos os processos rcu_
    'idle_',                # Exclui processos como 'idle_task' (se existirem e forem visíveis)
    'node',                 # Exclui processos Node.js (se você não quiser vê-los)
]

# Variáveis globais para cálculo de rede
LAST_NET_BYTES_SENT = 0
LAST_NET_BYTES_RECV = 0
LAST_NET_TIME = time.time()

# Lista de processos para manter referências e obter cpu_percent corretamente
ACTIVE_PROCESSES = {} 

# --- Funções Auxiliares ---
def create_progress_bar_textual(percent, max_bar_width):
    """Cria uma barra de progresso em texto."""
    if max_bar_width < 3: 
        return "[]"
    num_chars_for_bar = max_bar_width - 2
    num_hashes = int((percent / 100) * num_chars_for_bar)
    num_spaces = num_chars_for_bar - num_hashes
    
    num_hashes = max(0, num_hashes)
    num_spaces = max(0, num_spaces)
    
    return f"[{'#' * num_hashes}{' ' * num_spaces}]"

def get_cpu_temperature():
    """Tenta obter a temperatura da CPU."""
    try:
        result = subprocess.run(['sensors'], capture_output=True, text=True, check=True)
        temp_data = result.stdout
        
        match_package = re.search(r'Package id\s+\d+:\s+\+(\d+\.\d+)°C', temp_data)
        if match_package:
            return f"{float(match_package.group(1)):.1f}°C"

        match_core = re.search(r'Core\s+\d+:\s+\+(\d+\.\d+)°C', temp_data)
        if match_core:
            return f"{float(match_core.group(1)):.1f}°C"
            
        for line in temp_data.splitlines():
            match_generic = re.search(r'temp(?:\d+)?:\s+\+(\d+\.\d+)°C', line)
            if match_generic:
                return f"{float(match_generic.group(1)):.1f}°C"
                
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    
    try:
        thermal_zones = [d for d in os.listdir('/sys/class/thermal/') if d.startswith('thermal_zone')]
        for zone in thermal_zones:
            if os.path.exists(f'/sys/class/thermal/{zone}/type') and os.path.exists(f'/sys/class/thermal/{zone}/temp'):
                with open(f'/sys/class/thermal/{zone}/type', 'r') as f_type:
                    zone_type = f_type.read().strip()
                if 'cpu' in zone_type.lower() or 'pkg_temp' in zone_type.lower():
                    with open(f'/sys/class/thermal/{zone}/temp', 'r') as f_temp:
                        temp_milli_c = int(f_temp.read().strip())
                        return f"{temp_milli_c / 1000:.1f}°C"
    except Exception:
        pass
        
    return "N/A"

def format_uptime(seconds):
    """Formata segundos em horas, minutos e segundos."""
    if seconds is None:
        return "N/A"
    m, s = divmod(int(seconds), 60)
    h, m = divmod(int(m), 60)
    return f"{int(h)}h {int(m)}m {int(s)}s"

def get_xmrig_status():
    """Tenta obter o status detalhado do XMRig via API local."""
    XMRIG_API_URL = "http://127.0.0.1:9000/2/summary" # Ajuste a porta se você usou outra

    status = {
        "hashrate": "N/A",
        "shares_good": "N/A",
        "shares_total": "N/A",
        "uptime": "N/A",
        "pool_url": "N/A",
        "error": None
    }

    try:
        response = requests.get(XMRIG_API_URL, timeout=2) # Timeout para evitar travamento
        response.raise_for_status() # Lança exceção para status de erro (4xx ou 5xx)
        data = response.json()

        # Hashrate (10s avg)
        total_hashrate_10s = data.get('hashrate', {}).get('total', [0,0,0])[0] 
        if total_hashrate_10s is not None:
            status["hashrate"] = f"{total_hashrate_10s:.1f} H/s"
        
        # Shares
        status["shares_good"] = data.get('results', {}).get('shares_good', 'N/A')
        status["shares_total"] = data.get('results', {}).get('shares_total', 'N/A')

        # Uptime - Geralmente está no nível raiz da resposta
        uptime_seconds = data.get('uptime', None) 
        status["uptime"] = format_uptime(uptime_seconds)

        # Pool URL - CORRIGIDO NOVAMENTE: Agora trata 'connection' como um dicionário
        connection_data = data.get('connection', None)
        if connection_data and isinstance(connection_data, dict):
            # Extrai a chave 'pool' do dicionário 'connection'
            status["pool_url"] = connection_data.get('pool', 'N/A (chave "pool" não encontrada)')
        else:
            status["pool_url"] = "N/A (Conexão não encontrada ou formato inesperado na API)"

    except requests.exceptions.ConnectionError:
        status["error"] = "XMRig API Desativada/Não Acessível"
    except requests.exceptions.Timeout:
        status["error"] = "XMRig API: Timeout"
    except requests.exceptions.RequestException as e:
        status["error"] = f"Erro na requisição da API: {e}"
    except (KeyError, IndexError, TypeError) as e:
        status["error"] = f"Estrutura da API XMRig inesperada: {e}"
    except Exception as e:
        status["error"] = f"Erro desconhecido da API: {e}"
        
    return status

# --- Funções de Coleta de Dados do Sistema ---
def get_cpu_info(max_bar_width=20):
    """Coleta informações da CPU."""
    cpu_percent_per_core = psutil.cpu_percent(interval=None, percpu=True) 
    total_cpu_percent = sum(cpu_percent_per_core) / len(cpu_percent_per_core) if cpu_percent_per_core else 0
    
    cpu_name = getattr(get_cpu_info, '_cpu_name', "Não disponível")
    if cpu_name == "Não disponível":
        try:
            with open('/proc/cpuinfo') as f:
                for line in f:
                    if 'model name' in line:
                        cpu_name = line.split(': ')[1].strip()
                        setattr(get_cpu_info, '_cpu_name', cpu_name)
                        break
        except Exception:
            pass

    cpu_bar = create_progress_bar_textual(total_cpu_percent, max_bar_width)
    cpu_temp = get_cpu_temperature()

    return {
        "name": cpu_name,
        "cores_threads": f"{NUM_CPUS_PHYSICAL} Cores / {NUM_CPUS_LOGICAL} Threads",
        "usage": f"{total_cpu_percent:.1f}% {cpu_bar}",
        "temp": f"{cpu_temp}"
    }

def get_memory_info(max_bar_width=20):
    """Coleta informações da memória RAM."""
    mem = psutil.virtual_memory()
    mem_bar = create_progress_bar_textual(mem.percent, max_bar_width)

    return {
        "total": f"{mem.total / (1024**3):.2f}G",
        "used": f"{mem.used / (1024**3):.2f}G ({mem.percent:.1f}%) {mem_bar}",
    }

def get_network_info():
    """Coleta informações de rede (upload/download speed)."""
    global LAST_NET_BYTES_SENT, LAST_NET_BYTES_RECV, LAST_NET_TIME

    current_net_io = psutil.net_io_counters()
    current_time = time.time()

    time_diff = current_time - LAST_NET_TIME
    if time_diff == 0:
        time_diff = 0.1 # Evita divisão por zero

    upload_speed_kbps = (current_net_io.bytes_sent - LAST_NET_BYTES_SENT) / time_diff / 1024
    download_speed_kbps = (current_net_io.bytes_recv - LAST_NET_BYTES_RECV) / time_diff / 1024

    LAST_NET_BYTES_SENT = current_net_io.bytes_sent
    LAST_NET_BYTES_RECV = current_net_io.bytes_recv
    LAST_NET_TIME = current_time

    return {
        "upload": f"{upload_speed_kbps:.1f} KB/s",
        "download": f"{download_speed_kbps:.1f} KB/s"
    }

def get_gpu_temperature():
    """Tenta obter a temperatura da GPU (requer nvidia-smi)."""
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=temperature.gpu', '--format=csv,noheader,nounits'], capture_output=True, text=True, check=True)
        temp = result.stdout.strip()
        return f"{temp}°C"
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    return "N/A"

def get_gpu_info(max_bar_width=20):
    """Coleta informações da GPU (requer nvidia-smi)."""
    gpu_name = "Não disponível"
    gpu_utilization = 0.0
    gpu_memory_used_gb = 0.0
    gpu_memory_total_gb = 0.0
    gpu_temp = get_gpu_temperature()

    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=name,utilization.gpu,memory.used,memory.total', '--format=csv,noheader,nounits'], capture_output=True, text=True, check=True)
        gpu_data = result.stdout.strip().split(',')
        
        gpu_name = f"{gpu_data[0].strip()}" 
        gpu_utilization = float(gpu_data[1])
        gpu_memory_used_gb = float(gpu_data[2]) / 1024 
        gpu_memory_total_gb = float(gpu_data[3]) / 1024 

        gpu_util_bar = create_progress_bar_textual(gpu_utilization, max_bar_width)
        
        memory_display = f"({gpu_memory_used_gb:.1f}G/{gpu_memory_total_gb:.1f}G)"

        return {
            "name": gpu_name,
            "memory": f"{memory_display}",
            "usage": f"{gpu_utilization:.1f}% {gpu_util_bar}",
            "temp": f"{gpu_temp}"
        }
    except FileNotFoundError:
        return {"error": "GPU: nvidia-smi não encontrado ou GPU não NVIDIA."}
    except Exception as e:
        return {"error": f"Erro GPU: {e}", "name": gpu_name, "temp": gpu_temp}

def get_disk_info():
    """
    Coleta informações de armazenamento para todos os discos montados,
    exibindo LABEL - Nome do Modelo para cada dispositivo.
    Exibe apenas a porcentagem de uso, sem barra de progresso.
    """
    partitions = psutil.disk_partitions(all=False)
    
    disk_devices = {} 
    
    partition_info_from_lsblk = {} 
    try:
        lsblk_output = subprocess.run(['lsblk', '-f', '-o', 'NAME,LABEL,MOUNTPOINT'], 
                                      capture_output=True, text=True, check=True).stdout
        
        lines = lsblk_output.splitlines()
        if not lines: return [] 

        data_lines = lines[1:]

        for line in data_lines:
            if not line.strip(): 
                continue
            
            match = re.match(r'(\S+)\s+([^\/]*?)(?:\s+(/\S.*))?$', line)
            if match:
                name = match.group(1)
                label = match.group(2).strip() if match.group(2) else None
                mountpoint_from_lsblk = match.group(3).strip() if match.group(3) else None

                if label and (label.lower() in ["ext4", "ntfs", "vfat", "iso9000", "xfs", "btrfs", "swap"] or label == ''):
                    label = None

                if name.startswith('loop') or name.startswith('snap') or name.startswith('dm-'): 
                    continue

                device_path = f"/dev/{name}" if not name.startswith('/dev/') else name
                partition_info_from_lsblk[device_path] = {
                    "label": label,
                    "mountpoint_from_lsblk": mountpoint_from_lsblk 
                }
            
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass 

    for p in partitions:
        try:
            if "snap" in p.device or "loop" in p.device or "tmpfs" in p.fstype or not os.path.exists(p.mountpoint):
                continue
            
            main_device = p.device
            if re.match(r'/dev/(nvme\d+n\d+p\d+)', p.device): 
                 main_device = re.match(r'(/dev/nvme\d+n\d+)', p.device).group(1)
            elif re.match(r'/dev/[a-z]+[0-9]+', p.device): 
                main_device = re.match(r'(/dev/[a-z]+)', p.device).group(1)
            
            if main_device not in disk_devices:
                disk_devices[main_device] = {
                    "partitions": [],
                    "total_space": 0,
                    "used_space": 0,
                    "main_mountpoint": None,
                    "device_model_name": None 
                }
            
            usage = psutil.disk_usage(p.mountpoint)
            
            lsblk_data_for_partition = partition_info_from_lsblk.get(p.device, {})
            partition_label = lsblk_data_for_partition.get("label", None)
            
            disk_devices[main_device]["partitions"].append({
                "mountpoint": p.mountpoint,
                "total": usage.total,
                "used": usage.used,
                "free": usage.free,
                "percent": usage.percent,
                "label": partition_label 
            })
            disk_devices[main_device]["total_space"] += usage.total
            disk_devices[main_device]["used_space"] += usage.used

            if disk_devices[main_device]["main_mountpoint"] is None or p.mountpoint == '/':
                disk_devices[main_device]["main_mountpoint"] = p.mountpoint

        except PermissionError:
            continue
        except FileNotFoundError:
            continue
        except Exception:
            continue
            
    formatted_disks = []
    for device, data in disk_devices.items():
        model_name = "Dispositivo Desconhecido" 
        
        try:
            lsblk_model_output = subprocess.run(['lsblk', '-no', 'MODEL', device], capture_output=True, text=True, check=False).stdout.strip()
            if lsblk_model_output and lsblk_model_output != 'MODEL':
                model_name = lsblk_model_output.splitlines()[0].strip()
            elif not lsblk_model_output and os.path.exists(f"/sys/class/block/{os.path.basename(device)}/device/model"):
                with open(f"/sys/class/block/{os.path.basename(device)}/device/model", "r") as f:
                    model_name = f.read().strip()
        except Exception:
            pass
        
        data["device_model_name"] = model_name

        main_disk_label_for_display = None
        
        if data["main_mountpoint"]:
            for p_info in data["partitions"]:
                if p_info["mountpoint"] == data["main_mountpoint"] and p_info["label"]:
                    if p_info["label"].strip() != "":
                        main_disk_label_for_display = p_info["label"].strip()
                        break
        
        if main_disk_label_for_display is None and data["partitions"]:
            for p_info in data["partitions"]:
                if p_info["label"] and p_info["label"].strip() != "":
                    main_disk_label_for_display = p_info["label"].strip()
                    break

        if main_disk_label_for_display is None:
            if data["main_mountpoint"] == '/':
                main_disk_label_for_display = "Sistema"
            elif data["main_mountpoint"] and data["main_mountpoint"].startswith('/'):
                temp_label = data["main_mountpoint"].split('/')[-1]
                main_disk_label_for_display = temp_label if temp_label else "Partição"
            else:
                main_disk_label_for_display = "Disco Local"

        friendly_display_name = f"{main_disk_label_for_display} - {model_name}"
        
        formatted_parts = []
        for p in data["partitions"]:
            part_label_display = f" ([cyan]{p['label']}[/cyan])" if p['label'] and p['label'].strip() != "" else ""
            formatted_parts.append(
                f"  [grey46]{p['mountpoint']}{part_label_display}[/grey46]: {p['used'] / (1024**3):.1f}G/{p['total'] / (1024**3):.1f}G ([green]{p['percent']:.1f}%[/green])"
            )
        
        total_percent = (data["used_space"] / data["total_space"] * 100) if data["total_space"] > 0 else 0
        total_usage_summary_string = f"Total: {data['used_space'] / (1024**3):.1f}G/{data['total_space'] / (1024**3):.1f}G ({total_percent:.1f}%)"

        formatted_disks.append({
            "device_name": friendly_display_name, 
            "total_usage_summary": total_usage_summary_string,
            "partitions_detail": formatted_parts,
            "raw_model_name": data["device_model_name"]
        })
    
    return formatted_disks

def get_top_processes_formatted(num_processes=7, exclude_users=None, exclude_names=None): 
    """
    Coleta e formata os processos mais intensivos em CPU,
    excluindo usuários e nomes de processos específicos (definidos externamente),
    e aplicando um filtro de uso mínimo de CPU.
    Retorna uma lista ordenada em tempo real por uso de CPU.
    """
    if exclude_users is None:
        exclude_users = []
    if exclude_names is None:
        exclude_names = []

    current_processes_data = []
    pids_to_remove = []

    # Atualiza processos existentes e coleta dados
    for pid, proc_obj in ACTIVE_PROCESSES.items():
        try:
            proc_obj_cpu_percent = proc_obj.cpu_percent(interval=None)
            proc_obj_mem_percent = proc_obj.memory_percent()
            
            try:
                username = proc_obj.username()
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                username = "N/A"

            try:
                name = proc_obj.name()
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                name = "N/A"

            # Aplicar exclusões com base nas listas fornecidas pelo usuário
            if username in exclude_users or any(ex_name.lower() in name.lower() for ex_name in exclude_names):
                continue
            
            # Filtro: Processo precisa ter pelo menos 0.2% de CPU para aparecer
            if proc_obj_cpu_percent < 0.2: 
                continue

            current_processes_data.append({
                'pid': proc_obj.pid,
                'name': name,
                'cpu_percent': proc_obj_cpu_percent,
                'memory_percent': proc_obj_mem_percent,
                'username': username
            })
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pids_to_remove.append(pid) 
        except Exception: 
            pids_to_remove.append(pid)

    # Remove processos inativos
    for pid in pids_to_remove:
        if pid in ACTIVE_PROCESSES:
            del ACTIVE_PROCESSES[pid] 

    # Adiciona novos processos à lista de rastreamento
    for proc in psutil.process_iter(['pid', 'name', 'username']):
        if proc.pid not in ACTIVE_PROCESSES:
            try:
                proc.cpu_percent(interval=None) 
                ACTIVE_PROCESSES[proc.pid] = proc
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
    
    # --- LÓGICA SIMPLIFICADA DE ORDENAÇÃO E LIMITE ---
    # Ordena todos os processos pelo uso de CPU em ordem decrescente
    sorted_processes = sorted(current_processes_data, key=lambda x: x.get('cpu_percent', 0), reverse=True)
    
    # Limita ao número desejado de processos (7)
    final_processes_to_display = sorted_processes[:num_processes]
    # --- FIM DA LÓGICA SIMPLIFICADA ---
    
    formatted_list = []
    for i, p in enumerate(final_processes_to_display):
        # A normalização de CPU% por núcleo é importante se você tem muitos núcleos.
        # Caso contrário, um processo usando 100% de um núcleo em uma CPU de 8 núcleos lógicos seria 100% no psutil,
        # mas apenas 12.5% do total da CPU.
        cpu_usage_normalized = (p.get('cpu_percent', 0.0) / NUM_CPUS_LOGICAL) if NUM_CPUS_LOGICAL > 0 else 0.0
        
        # Colorir processos root em vermelho, outros em magenta
        user_style = "magenta"
        if p['username'] == 'root':
            user_style = "red"
        
        formatted_list.append({
            "pid": str(p.get('pid', 'N/A')),
            "user": Text(p.get('username', 'N/A'), style=user_style), # Aplica estilo aqui
            "name": p.get('name', 'N/A'),
            "cpu_percent": f"{cpu_usage_normalized:.1f}%",
            "memory_percent": f"{p.get('memory_percent', 0.0):.1f}%"
        })
    
    return formatted_list

# --- Funções de Renderização para Rich ---

console = Console() 

def create_cpu_panel(max_bar_width=20):
    """Cria o painel de informações da CPU."""
    cpu_info = get_cpu_info(max_bar_width=max_bar_width) 
    content = (
        f"Nome: [bold cyan]{cpu_info['name']}[/bold cyan]\n"
        f"{cpu_info['cores_threads']}\n"
        f"Uso: [green]{cpu_info['usage']}[/green]\n"
        f"Temp: [yellow]{cpu_info['temp']}[/yellow]"
    )
    return Panel(content, title="[bold blue]CPU[/bold blue]", border_style="blue", expand=True)

def create_gpu_panel(max_bar_width=20):
    """Cria o painel de informações da GPU."""
    gpu_info = get_gpu_info(max_bar_width=max_bar_width)
    
    if "error" in gpu_info:
        content = f"[bold red]ERRO:[/bold red] {gpu_info['error']}"
        if "name" in gpu_info: content += f"\nNome: {gpu_info['name']}"
        if "temp" in gpu_info: content += f"\nTemp: {gpu_info['temp']}"
    else:
        content = (
            f"Nome: [bold magenta]{gpu_info['name']}[/bold magenta]\n"
            f"Memória: {gpu_info['memory']}\n"
            f"Uso: [green]{gpu_info['usage']}[/green]\n"
            f"Temp: [yellow]{gpu_info['temp']}[/yellow]"
        )
    return Panel(content, title="[bold magenta]GPU[/bold magenta]", border_style="magenta", expand=True)

def create_memory_panel(max_bar_width=20):
    """Cria o painel de informações da memória."""
    mem_info = get_memory_info(max_bar_width=max_bar_width)
    content = (
        f"Total: [bold green]{mem_info['total']}[/bold green]\n"
        f"Usada: [green]{mem_info['used']}[/green]" 
    )
    return Panel(content, title="[bold green]Memória[/bold green]", border_style="green", expand=True)

def create_network_panel():
    """Cria o painel de informações da rede."""
    net_info = get_network_info()
    content = (
        f"Upload: [bold yellow]{net_info['upload']}[/bold yellow]\n"
        f"Download: [bold yellow]{net_info['download']}[/bold yellow]"
    )
    return Panel(content, title="[bold yellow]Rede[/bold yellow]", border_style="yellow", expand=True)

def create_disk_panel():
    """Cria o painel de informações de armazenamento. Sem barras de progresso."""
    disk_info_list = get_disk_info()
    
    if not disk_info_list:
        content = "[bold red]Nenhum dispositivo de armazenamento encontrado ou acessível.[/bold red]"
    else:
        full_content_lines = []
        for disk in disk_info_list:
            full_content_lines.append(f"[bold cyan]{disk['device_name']}[/bold cyan]")
            full_content_lines.append(f"  {disk['total_usage_summary']}") 
            for partition_line in disk['partitions_detail']:
                full_content_lines.append(partition_line)
            full_content_lines.append("") 
        content = "\n".join(full_content_lines).strip()
        
    return Panel(content, title="[bold cyan]Armazenamento[/bold cyan]", border_style="cyan", expand=True)


def create_processes_table():
    """Cria a tabela de processos."""
    table_min_height = 4 # Cabeçalho da tabela + bordas (2 de borda + 2 para header/footer da tabela)
    
    num_processes_to_display = 7 # Um número razoável para a maioria das telas

    table = Table(title="[bold red]Processos[/bold red]", show_header=True, header_style="bold blue", expand=True)
    table.add_column("PID", style="cyan", no_wrap=True) 
    table.add_column("Usuário", style="magenta") # Estilo base, será sobrescrito se root
    table.add_column("Nome", style="green", ratio=1) 
    table.add_column("CPU%", style="yellow", justify="right", width=6) 
    table.add_column("Mem%", style="red", justify="right", width=6) 

    processes = get_top_processes_formatted(num_processes=num_processes_to_display, 
                                                 exclude_users=EXCLUDE_USERS, 
                                                 exclude_names=EXCLUDE_PROCESS_NAMES)

    for p in processes:
        table.add_row(
            p['pid'],
            p['user'], 
            p['name'],
            p['cpu_percent'],
            p['memory_percent']
        )
    return table

# --- Loop Principal de Monitoramento (usando rich.live.Live) ---
def run_monitor_rich():
    """Função principal para executar o monitor Rich."""
    with Live(screen=True, auto_refresh=True, refresh_per_second=2) as live:
        while True:
            # Largura disponível para barras de progresso
            estimated_panel_width = (console.width // 2) - 4 
            bar_width = max(5, estimated_panel_width - 30) 
            
            cpu_panel = create_cpu_panel(max_bar_width=bar_width)
            gpu_panel = create_gpu_panel(max_bar_width=bar_width)
            mem_panel = create_memory_panel(max_bar_width=bar_width)
            net_panel = create_network_panel()
            disk_panel = create_disk_panel() 
            processes_table = create_processes_table()

            # --- Obter status detalhado do XMRig ---
            xmrig_status = get_xmrig_status()
            
            # Montar o conteúdo do painel XMRig
            xmrig_content_lines = []
            if xmrig_status["error"]:
                xmrig_content_lines.append(f"[bold red]Erro:[/bold red] {xmrig_status['error']}")
            else:
                xmrig_content_lines.append(f"[bold white]Hashrate:[/bold white] [green]{xmrig_status['hashrate']}[/green]")
                xmrig_content_lines.append(f"[bold white]Shares Boas:[/bold white] [green]{xmrig_status['shares_good']}[/green]")
                xmrig_content_lines.append(f"[bold white]Shares Totais:[/bold white] {xmrig_status['shares_total']}")
                xmrig_content_lines.append(f"[bold white]Rodando há:[/bold white] [cyan]{xmrig_status['uptime']}[/cyan]")
                # Ajusta a exibição da URL para quebrar linha se for muito longa
                pool_url_text = xmrig_status['pool_url']
                if len(pool_url_text) > 40: # Exemplo: quebrar se tiver mais de 40 caracteres
                    pool_url_text = f"{pool_url_text[:37]}..."
                xmrig_content_lines.append(f"[bold white]Pool URL:[/bold white] [yellow]{pool_url_text}[/yellow]")

            xmrig_panel = Panel(
                "\n".join(xmrig_content_lines),
                title="[bold yellow]XMRig Status[/bold yellow]",
                border_style="yellow",
                expand=True
            )
            # --- FIM NOVO ---

            main_layout = Layout(name="root", ratio=1)
            
            # 1. Cabeçalho fixo (3 linhas)
            # 2. Área de monitores superiores
            # 3. Área de processos
            main_layout.split_column(
                Layout(name="header", size=3), 
                Layout(name="top_monitors_area", ratio=6), 
                Layout(name="processes_area", ratio=4)   
            )

            # Atualização do cabeçalho para incluir a instrução de saída
            header_text = Text("Zionsoftware System Monitor", style="bold white on blue", justify="center")
            main_layout["header"].update(Panel(header_text, style="blue", expand=True))

            # Ajusta o layout dos monitores superiores para DUAS colunas
            top_monitors_layout = Layout(name="top_monitors_inner")
            top_monitors_layout.split_row(
                Layout(name="col1_monitors", ratio=1),
                Layout(name="col2_monitors", ratio=1)
            )

            # Coluna 1: CPU, Memória, Armazenamento
            # Definindo altura fixa com 'size'
            top_monitors_layout["col1_monitors"].split_column(
                Layout(cpu_panel, size=8), 
                Layout(mem_panel, size=6), 
                Layout(disk_panel, ratio=1)  
            )

            # Coluna 2: GPU, Rede e XMRig Status
            # Definindo altura fixa com 'size'
            top_monitors_layout["col2_monitors"].split_column(
                Layout(gpu_panel, size=8), 
                Layout(net_panel, size=6), 
                Layout(xmrig_panel, ratio=1)   
            )
            
            main_layout["top_monitors_area"].update(top_monitors_layout)
            
            processes_panel_for_layout = Panel(processes_table, border_style="red", expand=True)
            main_layout["processes_area"].update(processes_panel_for_layout)

            live.update(main_layout)

            time.sleep(0.5) 

# --- Execução Principal ---
if __name__ == "__main__":
    initial_net_io = psutil.net_io_counters()
    LAST_NET_BYTES_SENT = initial_net_io.bytes_sent
    LAST_NET_BYTES_RECV = initial_net_io.bytes_recv
    LAST_NET_TIME = time.time()

    for proc in psutil.process_iter(['pid', 'name', 'username']):
        try:
            proc.cpu_percent(interval=None) 
            ACTIVE_PROCESSES[proc.pid] = proc
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    
    try:
        run_monitor_rich()
    except KeyboardInterrupt:
        console.print("[bold red]Monitoramento encerrado.[/bold red]")