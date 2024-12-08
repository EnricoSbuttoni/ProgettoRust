use std::collections::VecDeque;
use device_query::{DeviceQuery, DeviceState, MouseState};
use std::thread;
use std::time::Duration;
use std::io::{self, Write};

// Configurazione della tolleranza e dei parametri
const DTW_TOLERANCE: f64 = 10.0; // Maggiore precisione
const MIN_MOVE_DISTANCE: f64 = 10.0;
const MAX_POINTS: usize = 150; // Massimo numero di punti per il percorso
const RESAMPLE_POINTS: usize = 50; // Numero target di punti per il resampling
const SMOOTHING_FACTOR: f64 = 0.2; // Fattore di smoothing
const DTW_WINDOW: usize = 5; // Finestra Sakoe-Chiba

fn main() {
    let device_state = DeviceState::new();
    let mut mouse_path: Vec<(f64, f64)> = Vec::new();
    let mut saved_pattern: Option<Vec<(f64, f64)>> = None;

    println!("Traccia una figura con il mouse e premi INVIO per terminarla.");
    thread::sleep(Duration::from_secs(3));

    // Modalità di registrazione con interazione
    loop {
        let mouse: MouseState = device_state.get_mouse();
        let pos = (mouse.coords.0 as f64, mouse.coords.1 as f64);

        if mouse_path.is_empty() || distance(mouse_path.last().unwrap(), &pos) > MIN_MOVE_DISTANCE {
            mouse_path.push(pos);
        }

        // Verifica se si raggiunge il massimo numero di punti
        if mouse_path.len() >= MAX_POINTS {
            println!("Hai raggiunto il massimo numero di punti registrabili (150).");
            break;
        }

        // Controlla se l'utente preme INVIO per terminare
        if is_enter_pressed() {
            println!("Registrazione terminata dall'utente.");
            break;
        }

        thread::sleep(Duration::from_millis(50));
    }

    if mouse_path.is_empty() {
        println!("Nessun percorso registrato.");
        return;
    }

    // Normalizza e salva il pattern registrato
    let resampled_path = resample_path(mouse_path.clone(), RESAMPLE_POINTS);
    saved_pattern = Some(normalize_path(resampled_path));
    println!("Figura salvata! Ora in ascolto per replicarla...");

    // Mostra il pattern salvato
    if let Some(ref pattern) = saved_pattern {
        for (i, pos) in pattern.iter().enumerate() {
            println!("Punto normalizzato {}: ({:.2}, {:.2})", i + 1, pos.0, pos.1);
        }
    }

    println!("Premi INVIO per avviare la modalità di ascolto.");
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();

    // Modalità di ascolto
    let mut current_path: VecDeque<(f64, f64)> = VecDeque::new();
    let mut action_executed = false;

    loop {
        let mouse: MouseState = device_state.get_mouse();
        let pos = (mouse.coords.0 as f64, mouse.coords.1 as f64);

        if current_path.is_empty() || distance(current_path.back().unwrap(), &pos) > MIN_MOVE_DISTANCE {
            current_path.push_back(pos);
        }

        if current_path.len() >= MAX_POINTS {
            current_path.pop_front();
        }

        if current_path.len() >= 15 {
            if let Some(ref pattern) = saved_pattern {
                let resampled_path = resample_path(current_path.clone().into_iter().collect(), RESAMPLE_POINTS);
                let smoothed_path = smooth_path(resampled_path, SMOOTHING_FACTOR);
                let normalized_path = normalize_path(smoothed_path);

                // Logging per debug
                println!(
                    "Punti attuali: {}, Distanza DTW: {:.2}",
                    current_path.len(),
                    dtw_distance_with_window(&pattern, &normalized_path, DTW_WINDOW)
                );

                if !action_executed && dtw_distance_with_window(&pattern, &normalized_path, DTW_WINDOW) < DTW_TOLERANCE {
                    println!("Figura replicata! Azione eseguita.");
                    action_executed = true;
                    break;
                }
            }
            current_path.pop_front();
        }

        thread::sleep(Duration::from_millis(50));
    }
}

// Funzione per controllare se INVIO è stato premuto
fn is_enter_pressed() -> bool {
    use std::io::Read;
    let mut buffer = [0; 1];
    if let Ok(n) = io::stdin().read(&mut buffer) {
        return n > 0 && buffer[0] == b'\n';
    }
    false
}

// Funzione di resampling per creare punti equidistanti
fn resample_path(path: Vec<(f64, f64)>, target_points: usize) -> Vec<(f64, f64)> {
    let mut resampled = Vec::new();
    if path.len() < 2 {
        return path;
    }

    let mut distances = vec![0.0];
    for i in 1..path.len() {
        distances.push(distances[i - 1] + distance(&path[i - 1], &path[i]));
    }
    let total_length = *distances.last().unwrap();

    let step = total_length / (target_points as f64 - 1.0);
    let mut current_length = 0.0;
    let mut j = 0;

    for _ in 0..target_points {
        while j < distances.len() - 1 && distances[j + 1] < current_length {
            j += 1;
        }
        if j >= distances.len() - 1 {
            break;
        }

        let t = if distances[j + 1] == distances[j] {
            0.0
        } else {
            (current_length - distances[j]) / (distances[j + 1] - distances[j])
        };
        resampled.push((
            path[j].0 + t * (path[j + 1].0 - path[j].0),
            path[j].1 + t * (path[j + 1].1 - path[j].1),
        ));
        current_length += step;
    }

    resampled
}

// Altre funzioni di supporto (smoothing, normalizzazione, DTW, distanza euclidea) rimangono invariati

fn smooth_path(path: Vec<(f64, f64)>, smoothing_factor: f64) -> Vec<(f64, f64)> {
    if path.len() < 3 {
        return path;
    }

    let mut smoothed = Vec::new();
    smoothed.push(path[0]);

    for i in 1..path.len() - 1 {
        let prev = path[i - 1];
        let current = path[i];
        let next = path[i + 1];

        smoothed.push((
            current.0 * (1.0 - smoothing_factor) + (prev.0 + next.0) * 0.5 * smoothing_factor,
            current.1 * (1.0 - smoothing_factor) + (prev.1 + next.1) * 0.5 * smoothing_factor,
        ));
    }

    smoothed.push(path[path.len() - 1]);
    smoothed
}

fn normalize_path(path: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    let min_x = path.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let min_y = path.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_x = path.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let max_y = path.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);

    let scale = ((max_x - min_x).max(max_y - min_y)).max(1.0);

    path.into_iter()
        .map(|(x, y)| ((x - min_x) / scale, (y - min_y) / scale))
        .collect()
}

fn distance(p1: &(f64, f64), p2: &(f64, f64)) -> f64 {
    ((p1.0 - p2.0).powi(2) + (p1.1 - p2.1).powi(2)).sqrt()
}

fn dtw_distance_with_window(path1: &[(f64, f64)], path2: &[(f64, f64)], window: usize) -> f64 {
    let n = path1.len();
    let m = path2.len();

    let mut dtw: Vec<Vec<f64>> = vec![vec![f64::INFINITY; m]; n];
    let w = window.max((n as isize - m as isize).abs() as usize);

    for i in 0..n {
        for j in (i.saturating_sub(w))..((i + w).min(m)) {
            let cost = distance(&path1[i], &path2[j]);
            if i == 0 && j == 0 {
                dtw[i][j] = cost;
            } else if i == 0 {
                dtw[i][j] = cost + dtw[i][j - 1];
            } else if j == 0 {
                dtw[i][j] = cost + dtw[i - 1][j];
            } else {
                dtw[i][j] = cost + dtw[i - 1][j].min(dtw[i][j - 1]).min(dtw[i - 1][j - 1]);
            }
        }
    }

    dtw[n - 1][m - 1]
}

