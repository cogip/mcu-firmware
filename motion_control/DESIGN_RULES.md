# Motion Control Design Rules

## Architecture Générale

### Vue d'ensemble

Le système de contrôle du mouvement utilise une architecture hiérarchique avec :
- **PlatformEngine** : Interface avec le matériel (localisation, moteurs), exécute la boucle de contrôle
- **MetaController** : Orchestre plusieurs contrôleurs (exécution séquentielle/parallèle)
- **Controllers** : Algorithmes de contrôle individuels (PID, filtres, etc.)
- **ControllersIO** : Magasin clé-valeur partagé pour la communication entre contrôleurs

### Flux de données

```
prepare_inputs()  → ControllersIO
                    ↓
               Controllers (lecture/écriture dans ControllersIO)
                    ↓
process_outputs() ← ControllersIO
```

### Deux chaînes principales

#### 1. QUADPID Chain (legacy)
- Contrôle simple sans feedforward
- PID pose directement vers PID vitesse
- Utilisé historiquement

#### 2. QUADPID_FEEDFORWARD Chain (actuel)
- Profils de mouvement feedforward
- Deux modes via ConditionalSwitch :
  - **Mode Feedforward** : Suit un profil de vitesse précalculé + correction PID tracking
  - **Mode Direct** : PID position direct (quand profil invalidé)

## Chaînes de contrôle Feedforward

### ConditionalSwitch

Sélectionne entre deux chaînes selon une condition booléenne :
- `linear_invalidate_profile` / `angular_invalidate_profile`
- **true** → mode direct (PID position seul)
- **false** → mode feedforward (profil + correction)

### Mode Feedforward (invalidate_profile=false)

**Utilisé pendant** : ROTATE_TO_DIRECTION, ROTATE_TO_FINAL_ANGLE

**Chaîne** :
1. **ProfileFeedforward** : Génère vitesse cible depuis profil précalculé
2. **PosePID (tracker)** : Corrige l'erreur de suivi du profil
3. **Combiner** : Additionne feedforward + correction → `linear_speed_order` / `angular_speed_order`
4. **AntiBlocking** : Détecte blocage mécanique

**Clés I/O** :
- PosePID lit `linear_tracking_error` (erreur entre profil et position actuelle)
- PosePID écrit `linear_feedback_correction`
- Combiner écrit `linear_speed_order`

### Mode Direct (invalidate_profile=true)

**Utilisé pendant** : MOVE_TO_POSITION (pour correction de cap pendant déplacement linéaire)

**Chaîne** :
1. **PosePID (corrector)** : Calcule vitesse depuis erreur de position
2. **AccelerationFilter** : Limite l'accélération
3. **DecelerationFilter** : Limite vitesse selon distance de freinage
4. **AntiBlocking** : Détecte blocage mécanique

**Clés I/O** :
- PosePID lit `linear_pose_error` (erreur réelle vers cible)
- PosePID écrit `linear_speed_order`
- AccelerationFilter lit/écrit `linear_speed_order`
- DecelerationFilter lit/écrit `linear_speed_order`

## États de PoseStraightFilter

Machine d'états pour déplacements point-à-point :

1. **ROTATE_TO_DIRECTION** : Rotation initiale vers direction de déplacement
   - `linear_invalidate_profile = true` (vitesse = 0)
   - `angular_invalidate_profile = false` (profil feedforward)

2. **MOVE_TO_POSITION** : Déplacement linéaire vers cible
   - `linear_invalidate_profile = false` (profil feedforward pour translation)
   - `angular_invalidate_profile = true` (PID direct pour maintenir le cap)
   - **Important** : La correction angulaire PID permet de corriger la dérive de cap

3. **ROTATE_TO_FINAL_ANGLE** : Rotation finale vers orientation cible
   - `linear_invalidate_profile = true` (vitesse = 0)
   - `angular_invalidate_profile = false` (profil feedforward)
   - Pas de correction linéaire (accepte légère dérive de position)

4. **FINISHED** : Cible atteinte, génère événement `pose_reached`

## Controller Chain I/O Keys

### Speed Order Variables
**RÈGLE IMPORTANTE** : Les variables `linear_speed_order` et `angular_speed_order` doivent vivre tout au long de la chaîne de contrôleurs.

Chaque contrôleur dans la chaîne :
- **LIT** depuis `linear_speed_order` / `angular_speed_order`
- **MODIFIE** la valeur
- **ÉCRIT** dans `linear_speed_order` / `angular_speed_order`

**PAS de variables intermédiaires** comme `linear_acceleration_speed` ou `linear_corrector_speed`.

### Exemple de chaîne directe correcte

```cpp
// PosePID
.speed_order = "linear_speed_order"  // Écrit

// AccelerationFilter
.target_speed = "linear_speed_order",  // Lit
.output_speed = "linear_speed_order"   // Écrit (modifie en place)

// DecelerationFilter
.target_speed = "linear_speed_order",  // Lit
.output_speed = "linear_speed_order"   // Écrit (modifie en place)

// AntiBlocking
.speed_order = "linear_speed_order"    // Lit et écrit
```

### Flux de données

```
Mode direct (invalidate_profile=true):
PosePID → linear_speed_order
         ↓ (lit/modifie/écrit)
AccelerationFilter → linear_speed_order
                    ↓ (lit/modifie/écrit)
DecelerationFilter → linear_speed_order
                    ↓ (utilisé par moteurs)

Mode feedforward (invalidate_profile=false):
Combiner → linear_speed_order
         ↓ (utilisé par AntiBlocking et moteurs)
```

## Cohérence entre Linear et Angular

Les chaînes linéaire et angulaire doivent être **symétriques** :
- Même structure de contrôleurs
- Même logique d'I/O keys
- Seul le préfixe change : `linear_` vs `angular_`

## Logs de debug

Les logs de debug dans les filtres (AccelerationFilter, DecelerationFilter) doivent rester désactivés en production pour éviter la pollution des logs.

Si nécessaire pour le debug, utiliser un niveau LOG_DEBUG plutôt que LOG_INFO.
